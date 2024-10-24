#pragma once
#include <cstdint>
#include <cmath>

struct Point {
    // Fields
    float x;
    float y;

    // Constructor
    Point(float x_, float y_) : x(x_), y(y_) {}

    // Operators overload
    inline Point operator+(const Point& other) const {  // Point p3 = p1 + p2
        return Point(x + other.x, y + other.y);
    }

    inline Point& operator+=(const Point& other) {      // p1 += p2
        x += other.x;
        y += other.y;
        return *this;
    }

    inline Point operator-(const Point& other) const {  // Point p3 = p1 - p2
        return Point(x - other.x, y - other.y);
    }
};

struct Pose {
    // Fields
    float x;        // x-coordinate of the pose
    float y;        // y-coordinate of the pose
    float theta;    // Angle of the pose in radians

    // Constructor
    inline Pose(float x_=0, float y_=0, float theta_=0) :
        x(x_), y(y_), theta(theta_) {}

    // Operators overload
    inline Pose operator*(const Pose& other) const {    // Pose = Pose * Pose
        float s = sin(theta);
        float c = cos(theta);
        return Pose(x + c*other.x - s*other.y,
                    y + s*other.x + c*other.y,
                    theta + other.theta);
    }

    inline Point operator*(const Point& other) const {  // Point = Pose * Point
        float s = sin(theta);
        float c = cos(theta);
        return Point(x + c*other.x - s*other.y,
                     y + s*other.x + c*other.y);
    }

    // Methods
    inline Point translation() { return Point(x, y); }
}

struct IndexPair {
    // Fields
    int r;
    int c;

    // Constructor
    IndexPair(int r_=0, int c_=0) : (r_), c(c_) {}

    // Operators overload
    inline IndexPair operator+(const IndexPair& other) const {
        return IndexPair(r + other.r, c + other.c);
    }

    inline IndexPair operator-(const IndexPair& other) const {
        return IndexPair(r - other.r, c - other.c);
    }

    inline int operator*(const IndexPair& other) const {
        return r*r + c*c;
    }
}
