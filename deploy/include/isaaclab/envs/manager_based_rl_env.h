// Copyright (c) 2025, Unitree Robotics Co., Ltd.
// All rights reserved.

#pragma once

#include <eigen3/Eigen/Dense>
#include <yaml-cpp/yaml.h>
#include "isaaclab/manager/observation_manager.h"
#include "isaaclab/manager/action_manager.h"
#include "isaaclab/assets/articulation/articulation.h"
#include "isaaclab/algorithms/algorithms.h"
#include <iostream>
#include <chrono>
#include "isaaclab/utils/utils.h"
#include "isaaclab/utils/tick_logger.h"

namespace isaaclab
{

class ObservationManager;
class ActionManager;

class ManagerBasedRLEnv
{
public:
    // Constructor
    ManagerBasedRLEnv(YAML::Node cfg, std::shared_ptr<Articulation> robot_)
    :cfg(cfg), robot(std::move(robot_))
    {
        // Parse configuration
        this->step_dt = cfg["step_dt"].as<float>();
        robot->data.joint_ids_map = cfg["joint_ids_map"].as<std::vector<float>>();
        robot->data.joint_pos.resize(robot->data.joint_ids_map.size());
        robot->data.joint_vel.resize(robot->data.joint_ids_map.size());

        { // default joint positions
            auto default_joint_pos = cfg["default_joint_pos"].as<std::vector<float>>();
            robot->data.default_joint_pos = Eigen::VectorXf::Map(default_joint_pos.data(), default_joint_pos.size());
        }
        { // joint stiffness and damping
            robot->data.joint_stiffness = cfg["stiffness"].as<std::vector<float>>();
            robot->data.joint_damping = cfg["damping"].as<std::vector<float>>();
        }

        robot->update();

        // load managers
        action_manager = std::make_unique<ActionManager>(cfg["actions"], this);
        observation_manager = std::make_unique<ObservationManager>(cfg["observations"], this);
    }

    void reset()
    {
        global_phase = 0;
        episode_length = 0;
        robot->update();
        action_manager->reset();
        observation_manager->reset();
        // Reset RNN hidden state if policy is RNN-based
        if (auto* ort = dynamic_cast<isaaclab::OrtRunner*>(alg.get())) {
            ort->reset_hidden_state();
        }
    }

    std::vector<float> encode(std::unordered_map<std::string, std::vector<float>> obs)
    {
        // input image from obs
        const auto input_it = obs.find("obs");
        if (input_it == obs.end()) {
            throw std::runtime_error("Policy observation group 'obs' is missing");
        }
        const std::vector<float>& input = input_it->second;

        // Update encoder_dim if not initialized or encoder changed
        if (encoder_dim == 0 && encoder) {
            encoder_dim = encoder->width * encoder->height * encoder->history;
        }

        int offset = input.size() - encoder_dim;

        if (offset <= 0) {
            throw std::runtime_error("Policy observation is shorter than the encoder image input");
        }

        // Resize depth_input_cache if needed
        if (depth_input_cache.size() != encoder_dim) {
            depth_input_cache.resize(encoder_dim);
        }

        if (proprio_input_cache.size() != offset) {
            proprio_input_cache.resize(offset);
        }

        // Copy depth data to cache
        for (int i = 0; i < encoder_dim; ++i) {
            depth_input_cache[i] = input[offset + i];
        }

        // Copy proprio input data
        for (int i = 0; i < offset; ++i) {
            proprio_input_cache[i] = input[i];
        }

        // Update image_obs_cache
        image_obs_cache["image"] = depth_input_cache;
        image_obs_cache["info"] = proprio_input_cache;

        std::vector<float> output = encoder->act(image_obs_cache);
        latent_cache = output;   // kept for the tick log; the encoder output is otherwise discarded

        // Resize new_input_cache if needed
        size_t new_size = offset + output.size();
        if (new_input_cache.size() != new_size) {
            new_input_cache.resize(new_size);
        }

        // Copy non-depth data
        for (int i = 0; i < offset; ++i) {
            new_input_cache[i] = input[i];
        }

        // Copy encoder output
        for (int i = 0; i < output.size(); ++i) {
            new_input_cache[offset + i] = output[i];
        }

        return new_input_cache;
    }

    void step()
    {
        episode_length += 1;
        const double t_mono = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - log_epoch_).count();

        robot->update();
        auto obs = observation_manager->compute();

        if (encoder) {
            auto new_input = encode(obs);
            obs["obs"] = new_input;
        }

        auto action = alg->act(obs);
        action_manager->process_action(action);

        if (log_requested_) log_tick(t_mono, action);
    }

    /// Ask for a per-tick recording of everything the policy consumed and produced.
    ///
    /// The schema cannot be fixed here: the observation and latent widths are only known once the
    /// first tick has run, so the file is opened lazily on the first `step()`. Call before
    /// entering the control loop.
    void enable_tick_log(const std::string& path)
    {
        log_path_ = path;
        log_requested_ = true;
        log_epoch_ = std::chrono::steady_clock::now();
    }

    void stop_tick_log() { tick_log_.stop(); }

    long tick_log_records() const { return static_cast<long>(tick_log_.recorded()); }

private:
    /// Build the record layout from the widths this policy actually uses, then open the file.
    void configure_tick_log(size_t action_dim)
    {
        auto* ort = dynamic_cast<isaaclab::OrtRunner*>(alg.get());
        const size_t hidden_dim = ort ? ort->hidden_state().size() : 0;
        const size_t joints = robot->data.joint_pos.size();
        tick_log_.configure({
            {"tick",        1,                              isaaclab::TickLogger::U64},
            {"t_mono",      1,                              isaaclab::TickLogger::F64},
            {"t_camera",    1,                              isaaclab::TickLogger::F64},
            {"info",        (uint32_t)proprio_input_cache.size(), isaaclab::TickLogger::F32},
            {"image",       (uint32_t)depth_input_cache.size(),   isaaclab::TickLogger::F32},
            {"latent",      (uint32_t)latent_cache.size(),   isaaclab::TickLogger::F32},
            {"hidden",      (uint32_t)hidden_dim,            isaaclab::TickLogger::F32},
            {"action_raw",  (uint32_t)action_dim,            isaaclab::TickLogger::F32},
            {"action_proc", (uint32_t)action_dim,            isaaclab::TickLogger::F32},
            {"joint_pos",   (uint32_t)joints,                isaaclab::TickLogger::F32},
            {"joint_vel",   (uint32_t)joints,                isaaclab::TickLogger::F32},
            {"joint_tau",   (uint32_t)joints,                isaaclab::TickLogger::F32},
            {"quat_wxyz",   4,                              isaaclab::TickLogger::F32},
            {"ang_vel",     3,                              isaaclab::TickLogger::F32},
        });
        if (!tick_log_.start(log_path_)) {
            printf("[tick_logger] could not open %s -- recording disabled\n", log_path_.c_str());
            log_requested_ = false;
            return;
        }
        printf("[tick_logger] recording %zu bytes/tick to %s\n", tick_log_.record_bytes(), log_path_.c_str());
    }

    void log_tick(double t_mono, const std::vector<float>& action_raw)
    {
        if (!tick_log_.configured()) configure_tick_log(action_raw.size());
        if (!log_requested_) return;

        uint8_t* slot = tick_log_.begin_record();
        if (!slot) return;   // slab overrun; the drop is counted in the file trailer

        auto* ort = dynamic_cast<isaaclab::OrtRunner*>(alg.get());
        const auto processed = action_manager->processed_actions();
        const auto& d = robot->data;
        // Eigen stores a quaternion as (x, y, z, w); the deploy convention everywhere else is
        // (w, x, y, z), so reorder rather than dumping coeffs() and confusing the parser.
        const float quat[4] = {d.root_quat_w.w(), d.root_quat_w.x(), d.root_quat_w.y(), d.root_quat_w.z()};

        isaaclab::RecordWriter w(slot);
        w.u64((uint64_t)episode_length);
        w.f64(t_mono);
        w.f64(d.last_camera_stamp);
        w.f32n(proprio_input_cache, proprio_input_cache.size());
        w.f32n(depth_input_cache, depth_input_cache.size());
        w.f32n(latent_cache, latent_cache.size());
        if (ort) w.f32n(ort->hidden_state(), ort->hidden_state().size());
        w.f32n(action_raw, action_raw.size());
        w.f32n(processed, processed.size());
        w.f32n(d.joint_pos.data(), d.joint_pos.size(), d.joint_pos.size());
        w.f32n(d.joint_vel.data(), d.joint_vel.size(), d.joint_vel.size());
        w.f32n(d.joint_tau.data(), d.joint_tau.size(), d.joint_tau.size());
        w.f32n(quat, 4, 4);
        w.f32n(d.root_ang_vel_b.data(), 3, 3);
        tick_log_.commit();
    }

public:
    float step_dt;

    YAML::Node cfg;

    std::unique_ptr<ObservationManager> observation_manager;
    std::unique_ptr<ActionManager> action_manager;
    std::shared_ptr<Articulation> robot;
    std::unique_ptr<Algorithms> alg;
    std::unique_ptr<EncoderRunner> encoder;
    long episode_length = 0;
    float global_phase = 0.0f;

    // Cached variables for encode function
    int encoder_dim = 0;
    std::vector<float> depth_input_cache;
    std::vector<float> proprio_input_cache;;
    std::unordered_map<std::string, std::vector<float>> image_obs_cache;
    std::vector<float> new_input_cache;
    std::vector<float> latent_cache;

    // Per-tick diagnostic recording. Off unless enable_tick_log() is called.
    isaaclab::TickLogger tick_log_;
    std::string log_path_;
    bool log_requested_ = false;
    std::chrono::steady_clock::time_point log_epoch_ = std::chrono::steady_clock::now();
};

};
