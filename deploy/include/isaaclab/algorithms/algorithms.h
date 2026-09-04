// Copyright (c) 2025, Unitree Robotics Co., Ltd.
// All rights reserved.

#pragma once

#include "onnxruntime_cxx_api.h"
#include <iostream>
#include <mutex>
#include <filesystem>
#include <cmath>
#include <atomic>
#include <array>
#include <spdlog/spdlog.h>

namespace isaaclab
{

class Algorithms
{
public:
    virtual std::vector<float> act(std::unordered_map<std::string, std::vector<float>> obs) = 0;

    std::vector<float> get_action()
    {
        std::lock_guard<std::mutex> lock(act_mtx_);
        return action;
    }

    std::vector<float> action;
protected:
    std::mutex act_mtx_;
};

class OrtRunner : public Algorithms
{
public:
    OrtRunner(std::string model_path)
    {
        // Init Model
        env = Ort::Env(ORT_LOGGING_LEVEL_WARNING, "onnx_model");

        auto available_providers = Ort::GetAvailableProviders();
        std::cout << "Available Providers: ";
        bool cuda_available = false;
        bool tensorrt_available = false;
        for (const auto& provider : available_providers) {
            std::cout << provider << " ";
            if (provider == "CUDAExecutionProvider") {
                cuda_available = true;
            }
            if (provider == "TensorrtExecutionProvider") {
                tensorrt_available = true;
            }
        }
        std::cout << std::endl;

        // TensorRT
        if (tensorrt_available) {
            // printf("Using TensorRT Execution Provider.\n");
            // OrtTensorRTProviderOptions trt_options{};
            // trt_options.device_id = 0;
            // trt_options.has_user_compute_stream = 0;
            // trt_options.user_compute_stream = nullptr;

            // session_options.AppendExecutionProvider_TensorRT(trt_options);
            // session_options.AppendExecutionProvider_TensorRT_V2(trt_options);
        }
        
        // CUDA
        if (cuda_available) {
            printf("Using CUDA Execution Provider.\n");
            OrtCUDAProviderOptions cuda_options{};
            cuda_options.device_id = 0;
            session_options.AppendExecutionProvider_CUDA(cuda_options);
        }
        
        session_options.SetGraphOptimizationLevel(ORT_ENABLE_EXTENDED);

        session = std::make_unique<Ort::Session>(env, model_path.c_str(), session_options);

        for (size_t i = 0; i < session->GetInputCount(); ++i) {
            Ort::TypeInfo input_type = session->GetInputTypeInfo(i);
            auto shape = input_type.GetTensorTypeAndShapeInfo().GetShape();
            for (auto& dimension : shape) {
                if (dimension < 0) dimension = 1;
            }
            input_shapes.push_back(std::move(shape));
            auto input_name = session->GetInputNameAllocated(i, allocator);
            input_names.push_back(input_name.release());
        }

        for (const auto& shape : input_shapes) {
            size_t size = 1;
            for (const auto& dim : shape) {
                size *= dim;
            }
            input_sizes.push_back(size);
        }

        // Get output shape
        Ort::TypeInfo output_type = session->GetOutputTypeInfo(0);
        output_shape = output_type.GetTensorTypeAndShapeInfo().GetShape();
        for (auto& dimension : output_shape) {
            if (dimension < 0) dimension = 1;
        }
        auto output_name = session->GetOutputNameAllocated(0, allocator);
        output_names.push_back(output_name.release());

        action.resize(output_shape[1]);
    }

    std::vector<float> act(std::unordered_map<std::string, std::vector<float>> obs)
    {
        auto memory_info = Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeCPU);

        // // make sure all input names are in obs
        // for (const auto& name : input_names) {
        //     if (obs.find(name) == obs.end()) {
        //         throw std::runtime_error("Input name " + std::string(name) + " not found in observations.");
        //     }
        // }

        // Create input tensors
        std::vector<Ort::Value> input_tensors;
        for(int i(0); i<input_names.size(); ++i)
        {
            std::string name_str(input_names[i]);
            if (name_str == "input") {
                name_str = "obs";
            }
            auto& input_data = obs.at(name_str);
            auto input_tensor = Ort::Value::CreateTensor<float>(memory_info, input_data.data(), input_sizes[i], input_shapes[i].data(), input_shapes[i].size());
            input_tensors.push_back(std::move(input_tensor));
        }

        // Run the model
        auto output_tensor = session->Run(Ort::RunOptions{nullptr}, input_names.data(), input_tensors.data(), input_tensors.size(), output_names.data(), 1);

        // Copy output data
        auto floatarr = output_tensor.front().GetTensorMutableData<float>();
        std::lock_guard<std::mutex> lock(act_mtx_);
        std::memcpy(action.data(), floatarr, output_shape[1] * sizeof(float));
        return action;
    }

    const std::vector<std::vector<int64_t>>& input_shape() const { return input_shapes; }
    const std::vector<int64_t>& output_shape_info() const { return output_shape; }

private:
    Ort::Env env;
    Ort::SessionOptions session_options;
    std::unique_ptr<Ort::Session> session;
    Ort::AllocatorWithDefaultOptions allocator;

    std::vector<const char*> input_names;
    std::vector<const char*> output_names;

    std::vector<std::vector<int64_t>> input_shapes;
    std::vector<int64_t> input_sizes;
    std::vector<int64_t> output_shape;
};

class EncoderRunner : public OrtRunner
{
public:
    EncoderRunner(std::string model_path): OrtRunner(model_path)
    {
    }

public:
    int width;
    int height;

    int history;
};

class GrootRunner {
public:
    GrootRunner(const std::filesystem::path& policy_dir,
                const std::vector<float>& default_q)
    : default_q_(default_q),
      balance_(policy_dir / "GR00T-WholeBodyControl-Balance.onnx"),
      walk_(policy_dir / "GR00T-WholeBodyControl-Walk.onnx") {
        if (default_q_.size() < 15) throw std::runtime_error("Groot default pose must contain 15 lower joints");
        validate(balance_, "balance");
        validate(walk_, "walk");
        latest_target_.assign(default_q_.begin(), default_q_.begin() + 15);
    }

    std::vector<float> act(const std::unordered_map<std::string, std::vector<float>>& observations,
                           const std::array<float, 3>& command) {
        const float speed = std::sqrt(command[0] * command[0] + command[1] * command[1] + command[2] * command[2]);
        const bool walk = speed >= 0.05f && !stand_;
        auto action = (walk ? walk_ : balance_).act(observations);
        std::lock_guard<std::mutex> lock(mutex_);
        previous_action_ = action;
        latest_target_.resize(15);
        for (size_t i = 0; i < 15; ++i) latest_target_[i] = default_q_[i] + action[i] * 0.25f;
        return action;
    }

    std::vector<float> latest_target() const { std::lock_guard<std::mutex> lock(mutex_); return latest_target_; }
    std::vector<float> previous_action() const { std::lock_guard<std::mutex> lock(mutex_); return previous_action_; }
    void set_stand(bool stand) { stand_ = stand; }

private:
    static void validate(const OrtRunner& runner, const char* name) {
        if (runner.input_shape().empty() || runner.input_shape().front().size() != 2 || runner.input_shape().front().back() != 516)
            throw std::runtime_error(std::string("Groot ") + name + " model input shape must be [1,516]");
        if (runner.output_shape_info().size() != 2 || runner.output_shape_info().back() != 15)
            throw std::runtime_error(std::string("Groot ") + name + " model output shape must be [1,15]");
    }

    std::vector<float> default_q_;
    OrtRunner balance_;
    OrtRunner walk_;
    mutable std::mutex mutex_;
    std::vector<float> latest_target_;
    std::vector<float> previous_action_ = std::vector<float>(15, 0.0f);
    std::atomic<bool> stand_{false};
};
};
