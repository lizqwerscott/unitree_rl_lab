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
#include "isaaclab/utils/utils.h"

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
    }

    std::vector<float> encode(std::unordered_map<std::string, std::vector<float>> obs)
    {
        // input image from obs
        std::vector<float> input = obs["obs"];

        // Update encoder_dim if not initialized or encoder changed
        if (encoder_dim == 0 && encoder) {
            encoder_dim = encoder->width * encoder->height * encoder->history;
        }

        int offset = input.size() - encoder_dim;

        if (offset <= 0) {
            printf("error: %ld, %d ,%d\n", input.size(), encoder_dim, offset);
            return std::vector<float>();
        }

        // Resize depth_input_cache if needed
        if (depth_input_cache.size() != encoder_dim) {
            depth_input_cache.resize(encoder_dim);
        }

        // Copy depth data to cache
        for (int i = 0; i < encoder_dim; ++i) {
            depth_input_cache[i] = input[offset + i];
        }

        // Update image_obs_cache
        image_obs_cache["obs"] = depth_input_cache;

        std::vector<float> output = encoder->act(image_obs_cache);

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
        robot->update();
        auto obs = observation_manager->compute();

        if (encoder) {
            auto new_input = encode(obs);
            obs["obs"] = new_input;
        }

        auto action = alg->act(obs);
        action_manager->process_action(action);
    }

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
    std::unordered_map<std::string, std::vector<float>> image_obs_cache;
    std::vector<float> new_input_cache;
};

};
