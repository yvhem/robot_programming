#pragma once

#include <iostream>
#include <cmath>
#include <stdexcept>
#include "linalg.h"
#include "canvas.h"

// Forward declaration
struct WorldItem;
struct World;

struct WorldItemVector {
    using ItemType = WorldItem*;
    
    // Fields
    int _size = 0;
    ItemType* _values = 0;

    // Operator overload
    inline const ItemType& operator[](int pos) const { // equivalent to at(pos)
        if (pos < 0 || pos >= _size) throw std::runtime_error("out of bounds");
        return _values[pos];
    }

    // Methods
    inline int size() const { return _size; }
    
    inline ItemType& at(int pos) {
        if (pos < 0 || pos >= _size) throw std::runtime_error("out of bounds");
        return _values[pos];
    }
    
    inline const ItemType& at(int pos) const { // equivalent to operator[]
        if (pos < 0 || pos >= _size) throw std::runtime_error("out of bounds");
        return _values[pos];
    } 
    
    void resize(int new_size);
    void pushBack(ItemType item); // Append and resize
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
    bool isDescendant(const WorldItem* ancestor) const; /* Iterative check */
    virtual bool canCollide(const WorldItem* other) const;
    Isometry2f poseInWorld() const;
    /* Check for intersections .*/
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
