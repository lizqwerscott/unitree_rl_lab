#include "FSM/State_RLBase.h"
#include "unitree_articulation.h"
#include "isaaclab/envs/mdp/observations/observations.h"
#include "isaaclab/envs/mdp/actions/joint_actions.h"
#include <unordered_map>
#include <filesystem>
#include <ctime>

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


REGISTER_OBSERVATION(depth_image)
{
    auto & robot = env->robot;
    auto & asset = env->robot;

    int history_skip_frames = params["history_skip_frames"].as<int>(1);
    int num_output_frames = params["num_output_frames"].as<int>(0);

    int history_length = params["history_length"].as<int>(1);

    int downsample_factor = std::max(history_skip_frames, 1);

    int frames = std::max(num_output_frames, 1);
    if (num_output_frames <= 0) {
        int distance_to_image_plane_noised = 37;
        frames = int((distance_to_image_plane_noised - 1) / downsample_factor + 1);
    }

    int real_downsample_factor = 3;

    int start_idx = -1 - real_downsample_factor * (frames - 1);
    std::vector<int> depth_obs_indices;
    if (history_length == 1) {
        // get last frame only
        depth_obs_indices.push_back(-1);
    } else {
        depth_obs_indices = linspace(start_idx, -1, frames);
    }


    std::vector<float> depth_obs;

    if (asset->data.depth_image_buffer != nullptr) {
        try {
            std::vector<std::vector<float>> depth_image = asset->data.depth_image_buffer->buffer_index(depth_obs_indices);
            for (int i = 0; i < depth_image.size(); ++i) {
                for (int j = 0; j < depth_image[i].size(); ++j) {
                    depth_obs.push_back(depth_image[i][j]);
                }
            }
        } catch (const std::out_of_range &e) {
            printf("Error accessing depth image buffer: %s\n", e.what());
        }
    }

    // depth_obs should be of size num_output_frames * width * height, if not, pad with zeros
    int expected_size = 32 * 18 * history_length;
    if (depth_obs.size() < expected_size || depth_obs.size() > expected_size) {
        printf("Warning: depth_obs size is %lu, expected %d. Padding with zeros.\n", depth_obs.size(), expected_size);
        depth_obs.resize(expected_size, 0.0f);
    }

	if (env->robot->data.depth_image_clear) {
		printf("empty depth input:\n");
        depth_obs.resize(expected_size, 0.0f);
		for (int i = 0; i < expected_size; ++i) {
			depth_obs[i] = 0.1f;
		}
	}

    return depth_obs;
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

    env->alg = std::make_unique<isaaclab::OrtRunner>(policy_dir / "exported" / "actor.onnx");

    auto* actor = dynamic_cast<isaaclab::OrtRunner*>(env->alg.get());
    if (!actor->is_rnn() || actor->input_size(0) != 224 || actor->input_size(1) != 256
        || actor->output_size(0) != 29 || actor->output_size(1) != 256) {
        throw std::runtime_error("Actor ONNX contract mismatch: expected 224+256 inputs and 29+256 outputs");
    }

    auto encoder_path = policy_dir / "exported" / "0-depth_encoder.onnx";
    if (!std::filesystem::exists(encoder_path)) {
        encoder_path = policy_dir / "exported" / "0-depth_image_encoder.onnx";
    }
    if (!std::filesystem::exists(encoder_path)) {
        throw std::runtime_error("Depth encoder ONNX not found in " + (policy_dir / "exported").string());
    }
    env->encoder = std::make_unique<isaaclab::EncoderRunner>(encoder_path);
    if (env->encoder->input_size(0) != 576 || env->encoder->input_size(1) != 96
        || env->encoder->output_size(0) != 128) {
        throw std::runtime_error("Depth encoder ONNX contract mismatch: expected 576+96 inputs and 128 output");
    }
    env->encoder->width = 32;
    env->encoder->height = 18;
    env->encoder->history = 1;

    // Per-tick diagnostic recording. Opt-in through the FSM config:
    //
    //   Velocity:
    //     tick_log_dir: ./tick_logs      # omit or leave empty to record nothing
    //
    // The writing happens on a background thread, so the 50 Hz policy loop is unaffected. One file
    // per entry into this state, named by wall-clock start time; parse it with
    // `deploy/scripts/read_tick_log.py`.
    if (cfg["tick_log_dir"] && !cfg["tick_log_dir"].as<std::string>().empty()) {
        const auto dir = std::filesystem::path(cfg["tick_log_dir"].as<std::string>());
        std::error_code ec;
        std::filesystem::create_directories(dir, ec);
        if (ec) {
            spdlog::warn("tick_log_dir '{}' is not usable ({}); recording disabled", dir.string(), ec.message());
        } else {
            const auto now = std::time(nullptr);
            char stamp[32];
            std::strftime(stamp, sizeof(stamp), "%Y%m%d_%H%M%S", std::localtime(&now));
            const auto path = dir / ("tick_" + std::string(stamp) + ".bin");
            env->enable_tick_log(path.string());
            spdlog::info("tick log -> {}", path.string());
        }
    }

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
    std::vector<float> joint_signs = {
            1,
            1,
            -1,
            1,
            1,
            -1,
            1,
            1,
            -1,
            1,
            1,
            1,
            1,
            1,
            1,
            1,
            1,
            1,
            1,
            1,
            1,
            1,
            1,
            1,
            1,
            1,
            1,
            1,
            1,
        };
    auto action = env->action_manager->processed_actions();
    for(int i(0); i < env->robot->data.joint_ids_map.size(); i++) {
        lowcmd->msg_.motor_cmd()[env->robot->data.joint_ids_map[i]].q() = action[i] * joint_signs[i];
    }
}
