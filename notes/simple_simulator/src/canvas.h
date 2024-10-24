#pragma once
#include "opencv2/opencv.hpp"
#include <Eigen/Geometry>

class Canvas {
public:
    // Fields
    cv::Mat display_image;

    // Methods
    void putGridMap(const uint8_t* grid, int rows, int cols, float resolution);
    void putPoint(float x, float y);
    void putLine(float x0, float y0, float x1, float y1);

protected:
    // Fields
    Eigen::Transform2f _w2s = Eigen::Transform2f::Identity();
    Eigen::Transform2f _s2w = Eigen::Transform2f::Identity();

    // Methods
    virtual void onClick(int r, int c, int button);
    virtual void onResize(int width, int height);
    static void _mouseCallBack(int event, int x, int y, int flags, void* userdata);
};
