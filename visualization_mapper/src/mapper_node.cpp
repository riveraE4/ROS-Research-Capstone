#include "visualization_mapper/mapper_node.hpp"
#include <fstream>
#include <iomanip>

MapperNode::MapperNode() : Node("visualization_mapper") {
    slam_map_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map",
        rclcpp::SensorDataQoS(),
        std::bind(&MapperNode::slam_display1, this, std::placeholders::_1)
    );

    // slam_pose_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    //     "/pose",
    //     rclcpp::SensorDataQoS(),
    //     std::bind(&MapperNode::slam_display2, this, std::placeholders::_1)
    // );

    //rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr slam_pose_;   

    // lidar_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    //     "/scan", 
    //     rclcpp::SensorDataQoS(),
    //     std::bind(&MapperNode::scan_callback, this, std::placeholders::_1));

    // odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    //     "/odom", 
    //     10,
    //     std::bind(&MapperNode::odom_callback, this, std::placeholders::_1));

    SDL_Init(SDL_INIT_VIDEO);
    window_ = SDL_CreateWindow("Mapper", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, screen_size_ * 2, screen_size_, 0);
    renderer_ = SDL_CreateRenderer(window_, -1, SDL_RENDERER_ACCELERATED);

    // occupancy_grid_.resize(GRID_WIDTH, std::vector<uint8_t>(GRID_HEIGHT, 0));
}

MapperNode::~MapperNode() {
    SDL_DestroyRenderer(renderer_);
    SDL_DestroyWindow(window_);
    SDL_Quit();
}

void MapperNode::slam_display1(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "New map data received!");
    dump_map_to_file(msg);
    visualize_map(msg);

    auto t = msg->info;
    auto d = msg->data;
    std::cout << "map.info: (w, h, resolution) = " << t.width << "," << t.height << "," << t.resolution << std::endl;
    std::cout << "map.info: (origin_x, origin_y, orig_theta) = " << t.origin.position.x << "," << t.origin.position.y << "," <<
        2.0 * std::atan2(t.origin.orientation.z, t.origin.orientation.w) << std::endl;
    // if (t.width * t.height >= 359) {
    //     std::cout << "map.data: ... = ";
    //     for (int i = 0; i < 10; ++i) {  //t.width * t.height
    //         //std::cout << (int8_t)d[i] << ",";
    //         printf("%d,", d[i]);
    //     }
    //     std::cout << std::endl;
    // }
}

void MapperNode::dump_map_to_file(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    std::ofstream map_file("map_data.txt");

    auto info = msg->info;
    map_file << "Map Metadata:\n";
    map_file << "Width: " << info.width << " pixels\n";
    map_file << "Height: " << info.height << " pixels\n";
    map_file << "Resolution : " << info.resolution << " m/pixel\n";
    map_file << "Origin Position: (" << info.origin.position.x << ", " << info.origin.position.y << ", " << info.origin.position.z << ")\n";
    map_file << "Map Data (Occupancy Values):\n";
    map_file << "(-1: Unknown, 0: Free, 1: Occupied)\n";

    for (int y = 0; y < info.height; y++) {
        for (int x = 0; x < info.width; x++) {
            int index = y * info.width + x;
            map_file << std::setw(3) << (int)msg->data[index] << ",";
        }
        map_file << "\n";
    }

    map_file.close();
    RCLCPP_INFO(this->get_logger(), "Map data saved to map_data.txt");
}

void MapperNode::visualize_map(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    SDL_SetRenderDrawColor(renderer_, 30, 30, 30, 255);  
    SDL_RenderClear(renderer_);

    auto info = msg->info;
    float resolution = info.resolution;
    int map_width = info.width;
    int map_height = info.height;
    float scale_x = (float)screen_size_ / (map_width * resolution);
    float scale_y = (float)screen_size_ / (map_height * resolution);
    float scale = std::min(scale_x, scale_y);

    // Draw each cell
    for (int y = 0; y < map_height; y++) {
        for (int x = 0; x < map_width; x++) {
            int index = y * map_width + x;
            int8_t value = msg->data[index];

            // Set color based on occupancy
            if (value == -1) {        // Unknown
                SDL_SetRenderDrawColor(renderer_, COLOR_UNKNOWN.r, COLOR_UNKNOWN.g, COLOR_UNKNOWN.b, COLOR_UNKNOWN.a);
            } else if (value == 0) {  // Free
                SDL_SetRenderDrawColor(renderer_, COLOR_FREE.r, COLOR_FREE.g, COLOR_FREE.b, COLOR_FREE.a);
            } else {                  // Occupied
                SDL_SetRenderDrawColor(renderer_, COLOR_OCCUPIED.r, COLOR_OCCUPIED.g, COLOR_OCCUPIED.b, COLOR_OCCUPIED.a);
            }

            // Calculate pixel coordinates (centered)
            int pixel_x = (screen_size_/2) + (x - map_width/2) * resolution * scale;
            int pixel_y = (screen_size_/2) + (y - map_height/2) * resolution * scale;
            int cell_size = std::max(1, (int)(resolution * scale));

            // Draw cell
            SDL_Rect cell = {pixel_x, pixel_y, cell_size, cell_size};
            SDL_RenderFillRect(renderer_, &cell);
        }
    }

    // Draw origin (red square)
    SDL_SetRenderDrawColor(renderer_, COLOR_ORIGIN.r, COLOR_ORIGIN.g, COLOR_ORIGIN.b, COLOR_ORIGIN.a);
    SDL_Rect origin = {
        (screen_size_/2) - 5,
        (screen_size_/2) - 5,
        10, 10
    };
    SDL_RenderFillRect(renderer_, &origin);
    SDL_RenderPresent(renderer_);
    SDL_UpdateWindowSurface(window_);
}


void MapperNode::slam_display2(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Subscribed to /pose"); // debugging line
    auto p = msg->pose.pose.position;
    auto q = msg->pose.pose.orientation;
    std::cout << "pose.position: (x, y, z) = " << p.x << "," << p.y << "," << p.z << std::endl;
    std::cout << "pose.orientation: (theta) = " << 2.0 * std::atan2(q.z, q.w) << std::endl;
    RCLCPP_INFO(this->get_logger(), "pose.position: (x, y, z) = %.2f, %.2f, %.2f", p.x, p.y, p.z);
    RCLCPP_INFO(this->get_logger(), "pose.orientation (theta) = %.2f", 2.0 * std::atan2(q.z, q.w));

}

void MapperNode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    robot_x_ = msg->pose.pose.position.x;
    robot_y_ = msg->pose.pose.position.y;
    robot_z_ = msg->pose.pose.position.z;
    auto q = msg->pose.pose.orientation;
    robot_theta_ = 2.0 * std::atan2(q.z, q.w); // flat-ground yaw
//    std::cout << "(robot_x_, robot_y_, robot_theta_) = " << robot_x_ << "," << robot_y_ << "," << robot_theta_ << std::endl;
}

void MapperNode::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    unique_pixel_points_.clear();
    current_scan_points_.clear();

    for (size_t i = 0; i < msg->ranges.size(); ++i) {
        float range = msg->ranges[i];
        if (range < msg->range_min || range > msg->range_max) continue;

        float angle = msg->angle_min + i * msg->angle_increment;
        float local_x = range * std::cos(angle);
        float local_y = range * std::sin(angle);

        //float global_x = robot_x_ + local_x * std::cos(robot_theta_) - local_y * std::sin(robot_theta_);
        //float global_y = robot_y_ + local_x * std::sin(robot_theta_) + local_y * std::cos(robot_theta_);
        float global_x; // = range * std::cos(angle - robot_theta_);
        float global_y; // = range * std::sin(angle - robot_theta_);

        // if robot_theta_ is within pos. side 0 - 180
        // angle (subtraction) = 0 - 180 radians / 0 -> PI
            // float global_x = range * std::cos(angle - robot_theta_);
            // float global_y = range * std::sin(angle - robot_theta_);
        // else robot_theta_ is within neg. side of 0 - -180    
        // angle (addition) = 0 to -180  radians/ 0 -> PI
            // float global_x = range * std::cos(angle + robot_theta_);
            // float global_y = range * std::sin(angle + robot_theta_);

        if (robot_theta_ >= 0) {
            global_x = range * std::cos(angle - robot_theta_);
            global_y = range * std::sin(angle - robot_theta_);
        } else {
            global_x = range * std::cos(angle + robot_theta_);
            global_y = range * std::sin(angle + robot_theta_);
        }

        

        // if (i == 0)
        // {
        //     std::cout << "(robot_x_, robot_y_, robot_z_, robot_theta_) = " << robot_x_ << "," << robot_y_ << "," << robot_z_ << "," << robot_theta_ << std::endl;
        //     std::cout << "(sin(robot_theta_), cos(robot_theta_)) = " << std::sin(robot_theta_) << "," << std::cos(robot_theta_) << std::endl;
        //     std::cout << "(range, angle) = " << range << "," << angle << std::endl;
        //     std::cout << "(local_x, local_y) = " << local_x << "," << local_y << std::endl;
        //     std::cout << "(global_x, global_y) = " << global_x << "," << global_y << std::endl;
        // }

        // int gx = static_cast<int>((global_x + (GRID_WIDTH * GRID_RES / 2)) / GRID_RES);
        // int gy = static_cast<int>((global_y + (GRID_HEIGHT * GRID_RES / 2)) / GRID_RES);
        // if (gx >= 0 && gx < GRID_WIDTH && gy >= 0 && gy < GRID_HEIGHT) {
        //     occupancy_grid_[gx][gy] = 2;
        // }

        int global_sx = static_cast<int>(screen_size_ / 2 + global_x * scale_);
        int global_sy = static_cast<int>(screen_size_ / 2 - global_y * scale_);
        unique_pixel_points_.push_back({global_sx, global_sy});

        int local_sx = static_cast<int>(screen_size_ / 2 + local_x * scale_);
        int local_sy = static_cast<int>(screen_size_ / 2 - local_y * scale_);
        current_scan_points_.push_back({local_sx, local_sy});

        
    }

    SDL_SetRenderDrawColor(renderer_, 0, 0, 0, 255);
    SDL_RenderClear(renderer_);

    //draw_map();
    // draw_global_view();
    // draw_local_view();

    SDL_RenderPresent(renderer_);
}

// void MapperNode::draw_grid_overlay(SDL_Rect viewport) {
//     SDL_RenderSetViewport(renderer_, &viewport);
//     SDL_SetRenderDrawColor(renderer_, 50, 50, 50, 255); // light gray

//     float step = 1.0f; // 1 meter grid
//     float half = screen_size_ / 2;
//     for (float x = -half / scale_; x <= half / scale_; x += step) {
//         int sx = static_cast<int>(half + x * scale_);
//         SDL_RenderDrawLine(renderer_, sx, 0, sx, screen_size_);
//     }
//     for (float y = -half / scale_; y <= half / scale_; y += step) {
//         int sy = static_cast<int>(half - y * scale_);
//         SDL_RenderDrawLine(renderer_, 0, sy, screen_size_, sy);
//     }
// }

// void MapperNode::draw_map() {
//     SDL_Rect global_viewport = {0, 0, screen_size_, screen_size_};
//     draw_grid_overlay(global_viewport);
//     SDL_RenderSetViewport(renderer_, &global_viewport);

//     SDL_SetRenderDrawColor(renderer_, 255, 255, 255, 255);
//     for (int x = 0; x < GRID_WIDTH; ++x) {
//         for (int y = 0; y < GRID_HEIGHT; ++y) {
//             if (occupancy_grid_[x][y] == 2) {
//                 float wx = (x - GRID_WIDTH / 2) * GRID_RES;
//                 float wy = (y - GRID_HEIGHT / 2) * GRID_RES;
//                 int sx = static_cast<int>(screen_size_ / 2 + wx * scale_);
//                 int sy = static_cast<int>(screen_size_ / 2 - wy * scale_);
//                 SDL_RenderDrawPoint(renderer_, sx, sy);
//             }
//         }
//     }

//     SDL_SetRenderDrawColor(renderer_, 0, 0, 255, 255);
//     int rx = static_cast<int>(screen_size_ / 2 + robot_x_ * scale_);
//     int ry = static_cast<int>(screen_size_ / 2 - robot_y_ * scale_);
//     SDL_RenderDrawLine(renderer_, rx, ry, rx + static_cast<int>(15 * std::cos(robot_theta_)), ry - static_cast<int>(15 * std::sin(robot_theta_)));
// }

// void MapperNode::draw_local_view() { // right side of the visual
//     SDL_Rect local_viewport = {screen_size_, 0, screen_size_, screen_size_};
//     //draw_grid_overlay(local_viewport);
//     SDL_RenderSetViewport(renderer_, &local_viewport);

//     SDL_SetRenderDrawColor(renderer_, 255, 255, 255, 255);
//     for (const auto& p : current_scan_points_) {
//         SDL_RenderDrawPoint(renderer_, p.x, p.y);
//     }

//     SDL_SetRenderDrawColor(renderer_, 0, 255, 0, 255);
//     int c = screen_size_ / 2;
//     SDL_RenderDrawLine(renderer_, c, c, c + static_cast<int>(15 * std::cos(robot_theta_)), c - static_cast<int>(15 * std::sin(robot_theta_)));
// }

// void MapperNode::draw_global_view() { // left side of the visual
//     SDL_Rect global_viewport = {0, 0, screen_size_, screen_size_};
//     //draw_grid_overlay(global_viewport);
//     SDL_RenderSetViewport(renderer_, &global_viewport);

//     SDL_SetRenderDrawColor(renderer_, 255, 255, 255, 255);
//     for (const auto& p : unique_pixel_points_) {
//         SDL_RenderDrawPoint(renderer_, p.x, p.y);
//     }

//     SDL_SetRenderDrawColor(renderer_, 0, 255, 0, 255);
//     int c = screen_size_ / 2;
//     SDL_RenderDrawLine(renderer_, c, c, c + static_cast<int>(15 * std::cos(robot_theta_)), c - static_cast<int>(15 * std::sin(robot_theta_)));
// }