#pragma once
#include <vector>

struct Point {
    float x;
    float y;
};

class MapStorage {
public:
    void add_point(float x, float y);
    const std::vector<Point>& get_points() const;

private:
    std::vector<Point> points_;
};
