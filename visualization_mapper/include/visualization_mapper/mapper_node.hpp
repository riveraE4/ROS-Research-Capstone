#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>


#include <SDL2/SDL.h>
#include <cmath>
#include <vector>
#include <unordered_set>
#include <cstdint>



struct PixelPoint {
    int x;
    int y;
    bool operator==(const PixelPoint& other) const {
        return x == other.x && y == other.y;
    }
};

struct PixelPointHash {
    std::size_t operator()(const PixelPoint& p) const {
        return std::hash<int>()(p.x) ^ (std::hash<int>()(p.y) << 1);
    }
};

class MapperNode : public rclcpp::Node {
public:
    MapperNode();
    ~MapperNode();

private:
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    //void draw_map();
    void draw_local_view();
    void draw_global_view();
    void slam_display1(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void slam_display2(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

    void visualize_map(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void dump_map_to_file(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

    const SDL_Color COLOR_UNKNOWN = {50, 50, 50, 255};    // Gray
    const SDL_Color COLOR_FREE = {255, 255, 255, 255};    // White
    const SDL_Color COLOR_OCCUPIED = {0, 0, 0, 255};      // Black
    const SDL_Color COLOR_ORIGIN = {255, 0, 0, 255};      // Red

    // void draw_grid_overlay(SDL_Rect viewport);


    // subscribers
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr slam_map_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr slam_pose_;   

    
    float robot_x_{0.0f}, robot_y_{0.0f}, robot_z_{0.0f}, robot_theta_{0.0f};

    SDL_Window* window_;
    SDL_Renderer* renderer_;
    const int screen_size_ = 800;
    const float scale_ = 50.0f;  // pixels per meter

    std::vector<PixelPoint> unique_pixel_points_;
    std::vector<PixelPoint> current_scan_points_;

    static constexpr int GRID_WIDTH = 1000;
    static constexpr int GRID_HEIGHT = 1000;
    static constexpr float GRID_RES = 0.05f; // meters per cell
    //std::vector<std::vector<uint8_t>> occupancy_grid_; 
};