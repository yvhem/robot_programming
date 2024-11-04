#pragma once

#include "linalg.h"
#include "grid.h"
#include "world.h"
#include "opencv2/opencv.hpp"
#include "canvas.h"

struct GridMap: public Grid, public WorldItem {
    float resolution = 1;       // meters per pixel
    float inv_resolution = 1;   // pixel per meter
    Vec2f grid_origin;
    Isometry2f _piw, _ipiw;
    cv::Mat cv_image;
    
    // Constructors
    GridMap(int rows, int cols, float res, WorldItem* p,
            const Isometry2f& pose_in_parent_);
    GridMap(const char* filename, float res, WorldItem* p,
            const Isometry2f& pose_in_parent_);
    
    // Methods
    inline Vec2f g2w(const Vec2f g) const {  // Grid -> World
        return _piw*(g - grid_origin)*resolution;
    }

    inline Vec2f w2g(const Vec2f w) const {  // World -> Grid
        return (_ipiw * w)*inv_resolution + grid_origin;
    }

    bool canCollide(const WorldItem* other) const override;
    bool collides(const WorldItem* item) const override;
    void draw(Canvas& dest) const override;
};
