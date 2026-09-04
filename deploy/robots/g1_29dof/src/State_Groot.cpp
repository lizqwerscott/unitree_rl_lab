#include "FSM/State_Groot.h"
#include "unitree_articulation.h"
#include "isaaclab/envs/mdp/observations/observations.h"
#include "isaaclab/envs/mdp/terminations.h"
#include <spdlog/spdlog.h>
#include <algorithm>
#include <filesystem>
#include <chrono>

namespace {
std::array<float, 29> yaml_array(const YAML::Node& node, const char* key) {
    std::array<float, 29> result{};
    if (node[key] && node[key].size() == 29) {
        const auto values = node[key].as<std::vector<float>>();
        std::copy(values.begin(), values.end(), result.begin());
    }
    return result;
}

const char* mode_name(groot::ControlMode mode) {
    switch (mode) {
        case groot::ControlMode::Gamepad: return "Gamepad";
        case groot::ControlMode::Navigation: return "Navigation";
        case groot::ControlMode::VLA: return "VLA";
    }
    return "Unknown";
}

const char* locomotion_name(groot::LocomotionMode mode) {
    return mode == groot::LocomotionMode::Auto ? "Auto" : "Stand";
}
}

State_Groot::State_Groot(int state_mode, std::string state_string)
: FSMState(state_mode, state_string), mode_manager(std::make_shared<groot::GrootModeManager>()) {
    gamepad_mode_key_ = unitree::common::dsl::Compile(*unitree::common::dsl::Parser("LB + X.on_pressed | LB.on_pressed + X").Parse());
    navigation_mode_key_ = unitree::common::dsl::Compile(*unitree::common::dsl::Parser("LB + Y.on_pressed | LB.on_pressed + Y").Parse());
    vla_mode_key_ = unitree::common::dsl::Compile(*unitree::common::dsl::Parser("LB + A.on_pressed | LB.on_pressed + A").Parse());
    auto_mode_key_ = unitree::common::dsl::Compile(*unitree::common::dsl::Parser("RB + Y.on_pressed | RB.on_pressed + Y").Parse());
    stand_mode_key_ = unitree::common::dsl::Compile(*unitree::common::dsl::Parser("RB + B.on_pressed | RB.on_pressed + B").Parse());
    const auto cfg = param::config["FSM"][state_string];
    const auto policy_dir = param::parser_policy_dir(cfg["policy_dir"].as<std::string>());
    const auto policy_cfg = YAML::LoadFile(policy_dir / "params" / "deploy.yaml");
    const auto urdf = std::filesystem::weakly_canonical(param::proj_dir / "../../assets/g1_29dof/main.urdf");
    limits_ = groot::load_urdf_limits(urdf);
    safe_home_ = yaml_array(cfg, "safe_home_q");
    if (safe_home_ == std::array<float, 29>{}) safe_home_ = yaml_array(policy_cfg, "safe_home_q");
    auto articulation = std::make_shared<unitree::BaseArticulation<LowState_t::SharedPtr>>(FSMState::lowstate);
    env = std::make_unique<isaaclab::ManagerBasedRLEnv>(policy_cfg, articulation);
    env->groot_mode_manager = mode_manager;
    env->groot_runner = std::make_unique<isaaclab::GrootRunner>(policy_dir / "exported", std::vector<float>(policy_cfg["groot_policy_default_q"].as<std::vector<float>>()));
    mode_manager->set_safe_home(safe_home_);
    const auto ranges = policy_cfg["commands"]["base_velocity"]["ranges"];
    mode_manager->set_command_limits(
        {ranges["lin_vel_x"][0].as<float>(), ranges["lin_vel_y"][0].as<float>(), ranges["ang_vel_z"][0].as<float>()},
        {ranges["lin_vel_x"][1].as<float>(), ranges["lin_vel_y"][1].as<float>(), ranges["ang_vel_z"][1].as<float>()});
    std::array<float, 29> defaults{};
    const auto default_values = policy_cfg["groot_policy_default_q"].as<std::vector<float>>();
    std::copy(default_values.begin(), default_values.end(), defaults.begin());
    mode_manager->set_policy_default(defaults);
    mode_manager->set_transition_duration(cfg["arm_transition"]["duration_ms"].as<float>(500.0f) / 1000.0f);
    receiver = std::make_unique<groot::RemoteCommandReceiver>(cfg["zmq"]["port"].as<int>(6002));
    registered_checks.emplace_back([&] { return isaaclab::mdp::bad_orientation(env.get(), 1.0); }, FSMStringMap.right.at("Passive"));
}

double State_Groot::steady_seconds() const {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

void State_Groot::enter() {
    for (size_t i = 0; i < env->robot->data.joint_stiffness.size(); ++i) {
        auto& motor = lowcmd->msg_.motor_cmd()[i];
        motor.kp() = env->robot->data.joint_stiffness[i];
        motor.kd() = env->robot->data.joint_damping[i];
        motor.dq() = 0; motor.tau() = 0;
    }
    env->reset();
    receiver->start();
    env->robot->update();
    for (size_t i = 0; i < last_published_q_.size(); ++i) last_published_q_[i] = env->robot->data.joint_pos[i];
    policy_thread_running_ = true;
    policy_thread = std::thread([this] {
        using clock = std::chrono::steady_clock;
        const auto dt = std::chrono::duration_cast<clock::duration>(std::chrono::duration<double>(0.02));
        auto next = clock::now();
        while (policy_thread_running_) {
            const auto now = steady_seconds();
            const float ly = FSMState::lowstate->joystick.ly();
            const float lx = FSMState::lowstate->joystick.lx();
            const float rx = FSMState::lowstate->joystick.rx();
            const auto command = mode_manager->velocity({ly, -lx, -rx}, now);
            env->last_velocity_command = {command.vx, command.vy, command.wz};
            env->command_override = true;
            env->step();
            next += dt;
            std::this_thread::sleep_until(next);
        }
    });
}

void State_Groot::run() {
    const auto now = steady_seconds();
    if (receiver) {
        groot::CommandSnapshot remote;
        if (receiver->latest(remote)) mode_manager->update_vla(remote);
    }
    if (FSMState::navcmd && !FSMState::navcmd->isTimeout()) {
        const auto& message = FSMState::navcmd->msg_;
        const groot::VelocityCommand navigation{message.linear().x(), message.linear().y(), message.angular().z()};
        if (!nav_initialized_ || navigation.vx != last_nav_command_.vx || navigation.vy != last_nav_command_.vy || navigation.wz != last_nav_command_.wz) {
            mode_manager->update_navigation(navigation);
            last_nav_command_ = navigation;
            nav_initialized_ = true;
        }
    }
    const std::string key = keyboard ? keyboard->key() : std::string{};
    const bool keyboard_pressed = keyboard && keyboard->on_pressed;
    const auto& joystick = FSMState::lowstate->joystick;
    std::array<float, 14> actual{};
    env->robot->update();
    for (size_t i = 0; i < actual.size(); ++i) actual[i] = env->robot->data.joint_pos[15 + i];
    const bool vla_requested = vla_mode_key_(joystick) || (keyboard_pressed && key == "3");
    const bool navigation_requested = navigation_mode_key_(joystick) || (keyboard_pressed && key == "2");
    const bool gamepad_requested = gamepad_mode_key_(joystick) || (keyboard_pressed && key == "1");
    const auto requested_mode = vla_requested ? groot::ControlMode::VLA
        : navigation_requested ? groot::ControlMode::Navigation
        : gamepad_requested ? groot::ControlMode::Gamepad
        : mode_manager->mode();
    if (requested_mode != mode_manager->mode()) {
        const auto old_mode = mode_manager->mode();
        if (mode_manager->request_mode(requested_mode, now, actual))
            spdlog::info("Groot: control mode {} -> {}", mode_name(old_mode), mode_name(requested_mode));
        else {
            const char* reason = requested_mode == groot::ControlMode::VLA
                ? "no fresh valid ZMQ packet on port 6002"
                : "no fresh navigation command";
            spdlog::warn("Groot: rejected control mode switch to {} ({})", mode_name(requested_mode), reason);
        }
    }
    const auto requested_locomotion = auto_mode_key_(joystick) || (keyboard_pressed && key == "m")
        ? groot::LocomotionMode::Auto
        : stand_mode_key_(joystick) || (keyboard_pressed && key == "p")
        ? groot::LocomotionMode::Stand
        : mode_manager->locomotion_mode();
    if (requested_locomotion != mode_manager->locomotion_mode()) {
        const auto old_mode = mode_manager->locomotion_mode();
        mode_manager->set_locomotion_mode(requested_locomotion);
        spdlog::info("Groot: locomotion mode {} -> {}", locomotion_name(old_mode), locomotion_name(requested_locomotion));
    }

    auto lower = env->groot_runner->latest_target();
    for (size_t i = 0; i < 15 && i < lower.size(); ++i) {
        lower[i] = std::clamp(lower[i], limits_[i].lower, limits_[i].upper);
        const float max_delta = limits_[i].velocity * 0.001f;
        lower[i] = std::clamp(lower[i], last_published_q_[i] - max_delta, last_published_q_[i] + max_delta);
        last_published_q_[i] = lower[i];
        lowcmd->msg_.motor_cmd()[i].q() = lower[i];
    }
    auto arm = mode_manager->arm_target(now);
    for (size_t i = 0; i < arm.size(); ++i) {
        const size_t joint = 15 + i;
        arm[i] = std::clamp(arm[i], limits_[joint].lower, limits_[joint].upper);
        const float max_delta = limits_[joint].velocity * 0.001f;
        arm[i] = std::clamp(arm[i], last_published_q_[joint] - max_delta, last_published_q_[joint] + max_delta);
        last_published_q_[joint] = arm[i];
        lowcmd->msg_.motor_cmd()[joint].q() = arm[i];
    }
}

void State_Groot::exit() {
    policy_thread_running_ = false;
    if (policy_thread.joinable()) policy_thread.join();
    receiver->stop();
}
