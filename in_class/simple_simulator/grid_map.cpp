#include "grid_map.h"
#include <cstring>

// Constructors
GridMap::GridMap(int rows, int cols, float res, WorldItem* p,
                 const Isometry2f& pose_in_parent_):
        Grid(rows, cols), WorldItem(p), resolution(res), 
        inv_resolution(1./res), grid_origin(rows/2, cols/2), _piw(poseInWorld()), 
        _ipiw(poseInWorld().inverse()) { 
    WorldItem::pose_in_parent = pose_in_parent_; 
}

/* Loads from image. */
GridMap::GridMap(const char* filename, float res, WorldItem* p,
                 const Isometry2f& pose_in_parent_): WorldItem(p) {
    pose_in_parent = pose_in_parent_;
    cv::Mat m = cv::imread(filename);
    if (m.rows == 0) throw runtime_error("unable to load image");
    cv::cvtColor(m, cv_image, cv::COLOR_BGR2GRAY);
    int size = cv_image.rows * cv_image.cols;
    resize(cv_image.rows, cv_image.cols);
    
    resolution = res;
    inv_resolution = 1./res;
    grid_origin = Vec2f(rows/2, cols/2);
    _piw = poseInWorld();
    _ipiw = _ipiw.inverse();
    memcpy(values, cv_image.data, size);
}

// Methods
bool GridMap::canCollide(const WorldItem* other) const {
    bool res = other->isDescendant(this);
    return res;
}

bool GridMap::collides(const WorldItem* item) const {
    if (!item->isDescendant(this)) return false;
    Vec2f gp = w2g(item->poseInWorld().t);
    int r0 = gp.x(), c0 = gp.y();
    int radius = (int)(item->radius * inv_resolution);
    int radius2 = radius * radius;
    for (int r=-radius; r<radius; ++r) {
        int rx = r + r0;
        for (int c=-radius; c<radius; ++c) {
            int cx = c + c0;
            if (!inside(rx, cx)) continue;
            if (at(rx, cx) < 127) return true;
        }
    }
    return true;
}

void GridMap::draw(Canvas& dest) const {
    Rotation2f Rt = _piw.R.inverse();
    Rotation2f sRt = Rt.scale(inv_resolution);
    Rotation2f s2Rt = Rt.scale(dest.resolution * inv_resolution);
    Vec2f sT = grid_origin - sRt*(dest.canvas_origin*dest.resolution + _ipiw.t);
    Vec2f t = _ipiw.t*(1./dest.resolution);
    for (int r=0; r<dest.rows(); ++r) {
        for (int c=0; c<dest.cols(); ++c) {
            Vec2f dest_v(r, c);
            Vec2f src_v = s2Rt*Vec2f(r, c) + sT;
            int src_r = src_v.x(), src_c = src_v.y();
            if (inside(src_r, src_c))
                dest.draw_image.at<uint8_t>(r,c) = 
                    cv_image.at<uint8_t>(src_r, src_c);
        }
    }
    WorldItem::draw(dest);
}
