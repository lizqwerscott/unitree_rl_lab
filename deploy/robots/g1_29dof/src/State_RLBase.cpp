#include "FSM/State_RLBase.h"
#include "unitree_articulation.h"
#include "isaaclab/envs/mdp/observations/observations.h"
#include "isaaclab/envs/mdp/actions/joint_actions.h"
#include <unordered_map>

namespace isaaclab
{
// keyboard velocity commands example
// change "velocity_commands" observation name in policy deploy.yaml to "keyboard_velocity_commands"
REGISTER_OBSERVATION(keyboard_velocity_commands)
{
    std::string key = FSMState::keyboard->key();
    static auto cfg = env->cfg["commands"]["base_velocity"]["ranges"];

    static std::unordered_map<std::string, std::vector<float>> key_commands = {
        {"w", {1.0f, 0.0f, 0.0f}},
        {"s", {-1.0f, 0.0f, 0.0f}},
        {"a", {0.0f, 1.0f, 0.0f}},
        {"d", {0.0f, -1.0f, 0.0f}},
        {"q", {0.0f, 0.0f, 1.0f}},
        {"e", {0.0f, 0.0f, -1.0f}}
    };
    std::vector<float> cmd = {0.0f, 0.0f, 0.0f};
    if (key_commands.find(key) != key_commands.end())
    {
        // TODO: smooth and limit the velocity commands
        cmd = key_commands[key];
    }
    return cmd;
}

REGISTER_OBSERVATION(generated_commands)
{
    std::vector<float> obs(3);
    auto & joystick = env->robot->data.joystick;

    const auto cfg = env->cfg["commands"]["base_velocity"]["ranges"];

    obs[0] = std::clamp(joystick->ly(), cfg["lin_vel_x"][0].as<float>(), cfg["lin_vel_x"][1].as<float>());
    obs[1] = std::clamp(-joystick->lx(), cfg["lin_vel_y"][0].as<float>(), cfg["lin_vel_y"][1].as<float>());
    obs[2] = std::clamp(-joystick->rx(), cfg["ang_vel_z"][0].as<float>(), cfg["ang_vel_z"][1].as<float>());

    return obs;
}

std::vector<int> linspace(int start, int stop, size_t num) {
    std::vector<int> result;
    if (num == 0) return result;
    if (num == 1) {
        result.push_back(start);
        return result;
    }

    result.reserve(num);

    double step = static_cast<double>(stop - start) / static_cast<double>(num - 1);
    for (size_t i = 0; i < num; ++i) {
        result.push_back(static_cast<int>(std::round(start + step * static_cast<double>(i))));
    }

    return result;
}

REGISTER_OBSERVATION(height_scan)
{
    auto & robot = env->robot;
    auto & asset = env->robot;

    std::vector<float> height_scan_obs = asset->data.height_scan_buffer;

    int expected_size = 33 * 21 * 3;
    if (height_scan_obs.size() < expected_size || height_scan_obs.size() > expected_size) {
        printf("Warning: height_scan_obs size is %lu, expected %d. Padding with zeros.\n", height_scan_obs.size(), expected_size);
        height_scan_obs.resize(expected_size, 0.0f);
    }

    return height_scan_obs;
}

}

State_RLBase::State_RLBase(int state_mode, std::string state_string)
: FSMState(state_mode, state_string)
{
    auto cfg = param::config["FSM"][state_string];
    auto policy_dir = param::parser_policy_dir(cfg["policy_dir"].as<std::string>());

    env = std::make_unique<isaaclab::ManagerBasedRLEnv>(
        YAML::LoadFile(policy_dir / "params" / "deploy.yaml"),
        std::make_shared<unitree::CameraArticulation<LowState_t::SharedPtr, CameraData_t::SharedPtr, TorsoImu_t::SharedPtr, Nav_Cmd_t::SharedPtr>>(FSMState::lowstate, FSMState::cameradata, FSMState::torsoimu, FSMState::navcmd)
    );

    env->alg = std::make_unique<isaaclab::OrtRunner>(policy_dir / "exported" / "policy.onnx");

    // 5s, dt is 0.001
    warmup_steps_max = 5 / 0.001;
    warmup_steps = warmup_steps_max;

    this->registered_checks.emplace_back(
        std::make_pair(
            [&]()->bool{ return isaaclab::mdp::bad_orientation(env.get(), 1.0); },
            FSMStringMap.right.at("Passive")
        )
    );
}

void State_RLBase::run()
{
    if (warmup_steps > 0) {
        warmup_steps--;
        return;
    }
    auto action = env->action_manager->processed_actions();
    for(int i(0); i < env->robot->data.joint_ids_map.size(); i++) {
        lowcmd->msg_.motor_cmd()[env->robot->data.joint_ids_map[i]].q() = action[i];
    }
}
