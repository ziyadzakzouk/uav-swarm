#pragma once
#include <cmath>

namespace uavswarm {

struct Vec2
{
    double x = 0, y = 0;

    Vec2() = default;
    Vec2(double x, double y) : x(x), y(y) {}

    Vec2 operator+(const Vec2 &o) const { return Vec2(x + o.x, y + o.y); }
    Vec2 operator-(const Vec2 &o) const { return Vec2(x - o.x, y - o.y); }
    Vec2 operator*(double k)      const { return Vec2(x * k, y * k); }
    double norm() const { return std::sqrt(x * x + y * y); }
};

}  // namespace uavswarm
