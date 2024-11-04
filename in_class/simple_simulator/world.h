#pragma once

#include <iostream>
#include <cmath>
#include <stdexcept>
#include "linalg.h"
#include "canvas.h"

struct WorldItem;
struct World;

struct WorldItemVector {
    // Fields
    using ItemType = WorldItem*;
    int _size = 0;
    ItemType* _values = 0;

    // Methods
    inline int size() const { return _size; }
    
    inline ItemType& at(int pos) {
        if (pos < 0 || pos >= _size) throw std::runtime_error("out of bounds");
        return _values[pos];
    }
    
    inline const ItemType& at(int pos) const {
        if (pos < 0 || pos >= _size) throw std::runtime_error("out of bounds");
        return _values[pos];
    }
    
    void resize(int new_size);
    void pushBack(ItemType item);
}

struct WorldItem {
    // Fields
    WorldItem* parent = nullptr;             // If 0 (nullptr) is the world
    WorldItem** children;
    Isometry2f pose_in_parent;
    float radius;

    // Constructors
    WorldItem(WorldItem* parent_=0);

    // Methods
    virtual World* getWorld();
    bool isDescendant(const WorldItem* ancestor) const;
    virtual bool canCollide(const WorldItem* other) const;
    Isometry2f poseInWorld() const;
    virtual bool collides(const WorldItem* other) const;
    virtual void timerTick(float dt);
    virtual void draw(Canvas& canvas) const;
};

struct World: public WorldItem {
    // Fields
    WorldItemVector items;

    // Methods
    World* getWorld() override;
    bool collides(const WorldItem* other) const override;
    const WorldItem* checkCollision(const WorldItem* current);
};
