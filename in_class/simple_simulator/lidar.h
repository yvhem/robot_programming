#pragma once

#include "world.h"
#include "grid_map.h"

struct LaserScan {
    // Fields
    float range_max=10, range_min=0.1;
    float angle_start=-M_PI/2, angle_end=M_PI/2;
    int num_beams;
    float* ranges;
    
    // Constructor
    LaserScan(int n_beams=181);
    
    // Destructor
    ~LaserScan();
}

struct Lidar: public WorldItem {
    // Fields
    LaserScan& scan;
    
    // Constructor
    Lidar(LaserScan& my_scan, WorldItem* p=0);

    // Methods
    const GridMap* getGridMap() const;
    virtual void timerTick(float dt) override;
    void draw(Canvas& canvas) const override;
}
