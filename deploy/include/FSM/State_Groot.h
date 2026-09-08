#pragma once

#include "FSMState.h"
#include "groot/GrootModeManager.h"
#include "groot/LowStateBroadcaster.h"
#include "groot/RemoteCommandReceiver.h"
#include "isaaclab/envs/manager_based_rl_env.h"
#include "groot/UrdfLimits.h"
#include <atomic>
#include <initializer_list>
#include <optional>
#include <thread>
#include <vector>

class State_Groot : public FSMState {
public:
    State_Groot(int state_mode, std::string state_string);
    void enter() override;
    void run() override;
    void exit() override;

private:
    enum class HeightAction {
        Up,
        Down,
        Reset,
    };

    struct KeyBinding {
        std::function<bool(const unitree::common::UnitreeJoystick &)> joystick;
        std::vector<std::string> keyboard_keys;
    };

    struct ControlModeKey {
        KeyBinding binding;
        groot::ControlMode mode;
    };

    struct LocomotionModeKey {
        KeyBinding binding;
        groot::LocomotionMode mode;
    };

    struct HeightKey {
        KeyBinding binding;
        HeightAction action;
    };

    KeyBinding make_key_binding(std::string joystick_expression, std::initializer_list<std::string> keyboard_keys = {});
    void register_control_mode_key(groot::ControlMode mode, std::string joystick_expression, std::initializer_list<std::string> keyboard_keys = {});
    void register_locomotion_mode_key(groot::LocomotionMode mode, std::string joystick_expression, std::initializer_list<std::string> keyboard_keys = {});
    void register_height_key(HeightAction action, std::string joystick_expression, std::initializer_list<std::string> keyboard_keys = {});
    bool key_active(const KeyBinding &binding, const unitree::common::UnitreeJoystick &joystick, const std::string &keyboard_key, bool keyboard_pressed) const;
    std::optional<groot::ControlMode> requested_control_mode(const unitree::common::UnitreeJoystick &joystick, const std::string &keyboard_key,
                                                              bool keyboard_pressed) const;
    std::optional<groot::LocomotionMode> requested_locomotion_mode(const unitree::common::UnitreeJoystick &joystick, const std::string &keyboard_key,
                                                                    bool keyboard_pressed) const;
    bool height_key_active(HeightAction action, const unitree::common::UnitreeJoystick &joystick, const std::string &keyboard_key,
                           bool keyboard_pressed) const;
    void update_navigation(double now);
    std::array<float, 14> read_actual_arm_positions();
    void update_control_mode(double now, const std::array<float, 14> &actual, const unitree::common::UnitreeJoystick &joystick,
                             const std::string &keyboard_key, bool keyboard_pressed);
    void update_height(double now, const unitree::common::UnitreeJoystick &joystick, const std::string &keyboard_key, bool keyboard_pressed);
    void update_remote_vla(double now, const std::array<float, 14> &actual);
    void update_locomotion_mode(const unitree::common::UnitreeJoystick &joystick, const std::string &keyboard_key, bool keyboard_pressed);
    void publish_targets(double now);

    std::unique_ptr<isaaclab::ManagerBasedRLEnv> env;
    std::shared_ptr<groot::GrootModeManager> mode_manager;
    std::unique_ptr<groot::RemoteCommandReceiver> receiver;
    std::unique_ptr<groot::LowStateBroadcaster> state_broadcaster;
    std::thread policy_thread;
    std::atomic<bool> policy_thread_running_{false};
    std::array<float, 29> safe_home_{};
    std::array<groot::JointLimit, 29> limits_{};
    std::array<float, 29> last_published_q_{};
    groot::VelocityCommand last_nav_command_{};
    bool nav_initialized_ = false;
    bool first_nav_logged_ = false;
    std::vector<ControlModeKey> control_mode_keys_;
    std::vector<LocomotionModeKey> locomotion_mode_keys_;
    std::vector<HeightKey> height_keys_;
    double next_height_adjust_time_ = 0.0;
    float last_height_logged_ = 0.74f;
    bool height_adjusting_ = false;
    double steady_seconds() const;
};

REGISTER_FSM(State_Groot)
