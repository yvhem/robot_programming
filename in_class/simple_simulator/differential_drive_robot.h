#pragma once

#include "world.h"

struct DifferentialDriveRobot: public WorldItem {
    // Fields
    float rot_vel = 0;
    float trans_vel = 0;
    
    // Constructor
    DifferentialDriveRobot(WorldItem* p=0): WorldItem(p) {}

    // Methods
    virtual void timerTick(float dt) override;
    void draw(Canvas& canvas) const override;
}
