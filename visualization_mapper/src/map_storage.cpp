#include "visualization_mapper/map_storage.hpp"

void MapStorage::add_point(float x, float y) {
    points_.emplace_back(Point{x, y});
}

const std::vector<Point>& MapStorage::get_points() const {
    return points_;
}
