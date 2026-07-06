#define UNITREE_CPP_DEBUG_DEX3_MAIN
#include "unitree_controller.hpp"
#include <iostream>
#include <chrono>
#include <thread>
#include <cstdlib>
#include <vector>

int main(int argc, char const* argv[]) {
    if (argc < 2) {
        std::cerr << "Usage: debug_dex3 <net_if>" << std::endl;
        return 1;
    }
    std::string net_if = argv[1];

    // Match the Python script's config
    std::vector<double> stiffness = {
        100, 100, 100, 150, 40, 40,
        100, 100, 100, 150, 40, 40,
        200, 200, 200,
        40, 40, 40, 40, 20, 20, 20,
        40, 40, 40, 40, 20, 20, 20
    };
    std::vector<double> damping = {
        5, 5, 5, 5, 2, 2,
        5, 5, 5, 5, 2, 2,
        6, 6, 6,
        2, 2, 2, 2, 2, 2, 2,
        2, 2, 2, 2, 2, 2, 2
    };

    UnitreeConfig config;
    config.net_if = net_if;
    config.control_dt = 0.02;
    config.msg_type = "hg";
    config.control_mode = ControlMode::POSITION;
    config.hand_type = "Dex-3";
    config.handstate_right_topic = "/lf/dex3/right/state";
    config.lowcmd_topic = "rt/lowcmd";
    config.lowstate_topic = "rt/lowstate";
    config.enable_odometry = true;
    config.sport_state_topic = "rt/odommodestate";
    config.stiffness = stiffness;
    config.damping = damping;
    config.num_dofs = 29;

    std::cout << "Creating UnitreeController..." << std::endl;
    UnitreeController controller(config);
    std::cout << "Controller created successfully." << std::endl;

    // Wait for data
    std::cout << "Waiting for self-check..." << std::endl;
    for (int i = 0; i < 50; i++) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        if (controller.self_check()) {
            std::cout << "Self-check passed!" << std::endl;
            break;
        }
        if (i % 10 == 9) {
            std::cout << "  still waiting..." << std::endl;
        }
    }

    // Send a test hand pose
    std::vector<double> r_pose = {0.0, -1.05, -1.7, 1.7, 1.8, 0.0, 0.0};
    std::cout << "Sending hand pose..." << std::endl;
    controller.step_hands({}, r_pose);

    std::cout << "Holding for 2 seconds..." << std::endl;
    for (int i = 0; i < 100; i++) {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        controller.step_hands({}, r_pose);
    }

    std::cout << "Shutting down..." << std::endl;
    controller.shutdown();

    std::cout << "Done." << std::endl;
    return 0;
}
