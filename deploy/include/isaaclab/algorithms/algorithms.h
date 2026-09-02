// Copyright (c) 2025, Unitree Robotics Co., Ltd.
// All rights reserved.

#pragma once

#include "onnxruntime_cxx_api.h"
#include <iostream>
#include <mutex>

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

            session_options.SetGraphOptimizationLevel(ORT_ENABLE_EXTENDED);
        }

        session = std::make_unique<Ort::Session>(env, model_path.c_str(), session_options);

        for (size_t i = 0; i < session->GetInputCount(); ++i) {
            Ort::TypeInfo input_type = session->GetInputTypeInfo(i);
            input_shapes.push_back(input_type.GetTensorTypeAndShapeInfo().GetShape());
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

        // Detect RNN mode: if any input name contains "hidden", activate RNN
        is_rnn_ = false;
        hidden_input_idx_ = 0;
        hidden_output_idx_ = 1;  // default: second output is hidden
        for (size_t i = 0; i < input_names.size(); ++i) {
            std::string name(input_names[i]);
            if (name.find("hidden") != std::string::npos) {
                is_rnn_ = true;
                hidden_input_idx_ = i;
                size_t hidden_size = 1;
                for (auto& dim : input_shapes[i]) {
                    hidden_size *= dim;
                }
                hidden_state_.resize(hidden_size, 0.0f);
                break;
            }
        }

        // Collect all output names
        output_names.clear();
        output_shapes.clear();
        for (size_t i = 0; i < session->GetOutputCount(); ++i) {
            Ort::TypeInfo output_type = session->GetOutputTypeInfo(i);
            auto shape = output_type.GetTensorTypeAndShapeInfo().GetShape();
            output_shapes.push_back(shape);
            size_t total = 1;
            for (auto& d : shape) total *= d;
            output_sizes.push_back(total);

            auto output_name = session->GetOutputNameAllocated(i, allocator);
            output_names.push_back(output_name.release());
        }

        if (is_rnn_) {
            // Detect which output corresponds to hidden state
            for (size_t i = 0; i < output_names.size(); ++i) {
                std::string name(output_names[i]);
                if (name.find("hidden") != std::string::npos) {
                    hidden_output_idx_ = i;
                    break;
                }
            }
        }

        // Resize action buffer — for RNN, skip hidden output
        auto find_action_dim = [this]() -> int64_t {
            for (size_t i = 0; i < output_shapes.size(); ++i) {
                if (is_rnn_ && std::string(output_names[i]).find("hidden") != std::string::npos)
                    continue;
                if (output_shapes[i].size() >= 2)
                    return output_shapes[i][1];
            }
            return output_shapes.empty() || output_shapes[0].size() < 2 ? 0 : output_shapes[0][1];
        };
        action.resize(find_action_dim());
    }

    std::vector<float> act(std::unordered_map<std::string, std::vector<float>> obs)
    {
        auto memory_info = Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeCPU);

        // Create input tensors
        std::vector<Ort::Value> input_tensors;
        for(int i(0); i<input_names.size(); ++i)
        {
            std::string name_str(input_names[i]);
            // printf("name: %s\n", name_str.c_str());

            if (name_str == "input") {
                name_str = "obs";
            }

            if (is_rnn_ && i == static_cast<int>(hidden_input_idx_)) {
                auto input_tensor = Ort::Value::CreateTensor<float>(
                    memory_info, hidden_state_.data(), hidden_state_.size(),
                    input_shapes[i].data(), input_shapes[i].size());
                input_tensors.push_back(std::move(input_tensor));
                continue;
            }

            auto& input_data = obs.at(name_str);
            auto input_tensor = Ort::Value::CreateTensor<float>(memory_info, input_data.data(), input_sizes[i], input_shapes[i].data(), input_shapes[i].size());
            input_tensors.push_back(std::move(input_tensor));
        }

        // Run the model — get all outputs
        std::vector<const char*> output_names_raw;
        for (auto& name : output_names) output_names_raw.push_back(name);

        auto output_tensors = session->Run(Ort::RunOptions{nullptr},
            input_names.data(), input_tensors.data(), input_tensors.size(),
            output_names_raw.data(), output_names_raw.size());

        // Find action output index (the one without "hidden" in name)
        int action_idx = 0;
        if (is_rnn_) {
            for (size_t i = 0; i < output_names.size(); ++i) {
                if (std::string(output_names[i]).find("hidden") == std::string::npos) {
                    action_idx = i;
                    break;
                }
            }
        }

        // Copy action data
        auto floatarr = output_tensors[action_idx].GetTensorMutableData<float>();
        size_t num_actions = 1;
        for (auto& dim : output_shapes[action_idx]) num_actions *= dim;

        std::lock_guard<std::mutex> lock(act_mtx_);
        action.resize(num_actions);
        std::memcpy(action.data(), floatarr, num_actions * sizeof(float));

        // Update hidden state if RNN
        if (is_rnn_) {
            auto hidden_data = output_tensors[hidden_output_idx_].GetTensorMutableData<float>();
            std::memcpy(hidden_state_.data(), hidden_data, hidden_state_.size() * sizeof(float));
        }

        return action;
    }

    void reset_hidden_state()
    {
        std::fill(hidden_state_.begin(), hidden_state_.end(), 0.0f);
    }

    /// Recurrent state carried between control ticks. Exposed for diagnostics: a drifting or
    /// diverging hidden state is one of the few remaining failure hypotheses and is otherwise
    /// completely unobservable from outside the process. Empty for a non-recurrent policy.
    const std::vector<float>& hidden_state() const { return hidden_state_; }
    bool is_rnn() const { return is_rnn_; }

private:
    Ort::Env env;
    Ort::SessionOptions session_options;
    std::unique_ptr<Ort::Session> session;
    Ort::AllocatorWithDefaultOptions allocator;

    std::vector<const char*> input_names;
    std::vector<const char*> output_names;

    std::vector<std::vector<int64_t>> input_shapes;
    std::vector<int64_t> input_sizes;
    std::vector<std::vector<int64_t>> output_shapes;
    std::vector<int64_t> output_sizes;   // 每个 output 的 total elements
    std::vector<float> hidden_state_;
    bool is_rnn_;
    size_t hidden_input_idx_;
    size_t hidden_output_idx_;
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
};
