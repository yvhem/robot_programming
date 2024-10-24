#pragma once
#include "world.h"

struct Robot : public WorldItem {
    // Fields
    float tv=0, rv=0;
    float radius;
    int radius_in_pixels;

    // Constructor
    Robot(World* world_, float radius_0.3) : 
        WorldItem(world_), radius(radius_),
        radius_in_pixels(radius_ * world_->inv_res) {}

    // Methods
    void timeTick(float dt) override;
    bool collidse(const Point& p);
    void draw() override;
};
