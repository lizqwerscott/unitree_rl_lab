// Copyright (c) 2025, Unitree Robotics Co., Ltd.
// All rights reserved.

#pragma once

#include <iostream>

#include <eigen3/Eigen/Dense>
#include <yaml-cpp/yaml.h>

#include "isaaclab/algorithms/algorithms.h"
#include "isaaclab/assets/articulation/articulation.h"
#include "isaaclab/manager/action_manager.h"
#include "isaaclab/manager/observation_manager.h"
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
        : cfg(cfg)
        , robot(std::move(robot_))
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

    void step()
    {
        episode_length += 1;
        robot->update();
        auto obs = observation_manager->compute();

        // split obs into multiple inputs for the policy according
        // split obs to obs, history, depth

        std::vector<int> obs_size;
        int obs_history_length = 10;

        // velocity
        obs_size.push_back(3);

        // base angle
        obs_size.push_back(3);

        // projected gravity
        obs_size.push_back(3);

        // joint pos
        obs_size.push_back(16);

        // joint vel
        obs_size.push_back(16);

        // last action
        obs_size.push_back(16);

        std::vector<float> inputs = obs["obs"];

        std::unordered_map<std::string, std::vector<float>> split_obs;

        int current_input_idx = 0;

        std::vector<float> obs_current;

        for (int i = 0; i < 3; ++i) {
            obs_current.push_back(inputs[current_input_idx]);
            current_input_idx++;
        }
        // Extract history observations
        std::vector<std::vector<float>> obs_historys;

        for (int i = 0; i < obs_size.size(); ++i) {
            int obs_dim = obs_size[i];

            std::vector<float> obs_history;
            for (int h = 0; h < obs_history_length; ++h) {
                for (int d = 0; d < obs_dim; ++d) {
                    obs_history.push_back(inputs[current_input_idx]);
                    current_input_idx++;
                }
            }
            obs_historys.push_back(obs_history);
        }

        std::vector<float> obs_history; // 570 values for history
        for (int h = 0; h < obs_history_length; h++) {
            for (int i = 0; i < obs_size.size(); ++i) {
                int obs_dim = obs_size[i];
                for (int d = 0; d < obs_dim; ++d) {
                    obs_history.push_back(obs_historys[i][h * obs_dim + d]);
                    if (h == 9) {
                        obs_current.push_back(obs_historys[i][h * obs_dim + d]);
                    }
                }
            }
        }

        std::vector<float> depth_image; // 64*64*2 = 8192 values for depth image
        for (int i = 0; i < 64 * 64 * 2; i++) {
            depth_image.push_back(inputs[current_input_idx]);
            current_input_idx++;
        }

        // Store in split_obs map
        split_obs["obs"] = obs_current;
        split_obs["history"] = obs_history;
        split_obs["depth"] = depth_image;

        auto action = alg->act(split_obs);
        action_manager->process_action(action);
    }

    float step_dt;

    YAML::Node cfg;

    std::unique_ptr<ObservationManager> observation_manager;
    std::unique_ptr<ActionManager> action_manager;
    std::shared_ptr<Articulation> robot;
    std::unique_ptr<Algorithms> alg;
    long episode_length = 0;
    float global_phase = 0.0f;
};

}; // namespace isaaclab
