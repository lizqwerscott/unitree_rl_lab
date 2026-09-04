#pragma once

#include "FSMState.h"
#include "groot/GrootModeManager.h"
#include "groot/RemoteCommandReceiver.h"
#include "isaaclab/envs/manager_based_rl_env.h"
#include "groot/UrdfLimits.h"
#include <atomic>
#include <thread>

class State_Groot : public FSMState {
public:
    State_Groot(int state_mode, std::string state_string);
    void enter() override;
    void run() override;
    void exit() override;

private:
    std::unique_ptr<isaaclab::ManagerBasedRLEnv> env;
    std::shared_ptr<groot::GrootModeManager> mode_manager;
    std::unique_ptr<groot::RemoteCommandReceiver> receiver;
    std::thread policy_thread;
    std::atomic<bool> policy_thread_running_{false};
    std::array<float, 29> safe_home_{};
    std::array<groot::JointLimit, 29> limits_{};
    std::array<float, 29> last_published_q_{};
    groot::VelocityCommand last_nav_command_{};
    bool nav_initialized_ = false;
    std::function<bool(const unitree::common::UnitreeJoystick&)> gamepad_mode_key_;
    std::function<bool(const unitree::common::UnitreeJoystick&)> navigation_mode_key_;
    std::function<bool(const unitree::common::UnitreeJoystick&)> vla_mode_key_;
    std::function<bool(const unitree::common::UnitreeJoystick&)> auto_mode_key_;
    std::function<bool(const unitree::common::UnitreeJoystick&)> stand_mode_key_;
    double steady_seconds() const;
};

REGISTER_FSM(State_Groot)
