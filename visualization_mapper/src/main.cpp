#include "visualization_mapper/mapper_node.hpp"
#include <rclcpp/rclcpp.hpp>
#include <cstdint>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MapperNode>();

    // Add SDL event loop
    SDL_Event e;
    bool quit = false;
    while (rclcpp::ok() && !quit) {
        while (SDL_PollEvent(&e)) {
            if (e.type == SDL_QUIT) quit = true;
        }
        rclcpp::spin_some(node);
        SDL_Delay(10);  // Prevent CPU overload
    }

    rclcpp::shutdown();
    return 0;
}
