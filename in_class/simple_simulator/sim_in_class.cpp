#include <iostream>
#include <cmath>
#include "linalg.h"

using namespace std;

struct GridMap {
    // Fields
    using CellType = uint8_t;
    int rows, cols;
    CellType** values;

    // Constructors
    GridMap(int rows=0, int cols=0) {
        resize(rows, cols);
    }

    // Destructor
    ~Grid() { clear(); }

    // Methods
    CellType& at(int r, int c) {
        return values[r][c];
    }

    const CellType& at(int r, int c) const {
        return values[r][c];
    }

    void resize(int r, int c) {
        if (r == rows && c == cols) return;
        clear();
        rows = r; cols = c;
        values = new CellType*[rows];
        for (int rr=0; rr<rows; ++rr) values[rr] = new CellType[cols]
    }

    void clear() {
        if (!values) return;
        for (int r=0; r<rows; ++r) delete[] values[r];
        delete[] values;
        values = 0; rows = 0; cols = 0;
    }
};

struct GridMapping {
    float resolution;       // meters per pixel
    float inv_resolution;   // pixel per meters
    float rows, cols;
    
    Vec2f g2w(const Vec2f g) {  // Grid -> World
        return g*resolution;
    }

    Vec2f w2g(const Vec2f w) {  // World -> Grid
        return w*inv_resolution + Vec2f(rows/2, cols/2);
    }
};

struct World;

struct WorldItem {
    // Fields
    WorldItem* parent;             // If 0 is the world
    WorldItem** children;
    int num_children;
    World* world;
    Isometry2f pose_in_parent;
    float radius;

    // Constructors
    WorldItem(WorldItem* parent_=0) {
        if (parent_) {
            parent = parent_;
            parent->addChildren(this);
        }
        WorldItem* r = getRoot();
        if (r == this) return;
        World* w = dynamics_cast<World*>(r); // try to cast WorldItem* to World*
        if (!w) throw std::runtime_error("male");
        w->addItemToAll(this);
    }

    // Methods
    Isometry2f poseInWorld() const { // Return the pose of an item in the world
        if (!parent) return pose_in_parent;
        return parent->poseInWorld() * pose_in_parent;
    }

    addChildren(WorldItem* child);

    virtual bool collides(const WorldItem* other) const {
        Isometry2f my_pose_in_world = poseInWorld();
        Isometry2f other_pose_in_world = other->poseInWorld();
        // Compute the pose of the other object wrt me
        Vec2f distance = (my_pose_in_world.t - other_pose_in_world.t).norm();
        return (distance < (radius + other->radius));
    };

    virtual void timerTick(float dt) {
        for (int i=0; i<num_children; ++i)
            if (children[i]) children[i]->timerTick(dt);
    }

    WorldItem* getRoot() {
        if (!parent) return this;
        return parent->getRoot();
    }

    int countDescendants() {
        int nds = 0;
        for (int i=0; i<num_children; ++i)
            if (children[i]) nds += children->countDescendants[i];
    }
};

struct World: public WorldItem {
    // Fields
    GridMap* grid;
    GridMapping grid_mapping;
    WorldItem** all_items;
    int num_all_items;
    addItemToAll(WorldItem*);

    // Methods
    bool collides(const WorldItem* other) const override {
        cerr << "AAAARGHHHHH" << endl;
        return false;
    }

    WorldItem* checkCollision(WorldItem* current) {
        for (int i=0; i<num_all_items; ++i) {
            WorldItem* o = all_items[i];
            if (o && o != current && !current->isDescendant(o) && 
                o->collides(current)) 
                    return o;
        }
        return nullptr;
    }
};

struct DifferentialDriveRobot: WorldItem {
    // Fields
    float rot_vel, trans_vel;
    
    // Methods
    void TimerTick(float dt) override {
        Isometry2f motion(dt*trans_vel, 0, dt*rot_vel);
        Isometry2f old_pose_in_parent = pose_in_parent;
        pose_in_parent = pose_in_parent*motion;
        World* w = getWorld();
        if (w->checkCollision(this)) pose_in_parent = old_pose_in_parent;
        WorldItem::timerTick();
    }
}

int main(int argc, char** argv) {
    Vec2f v1;
    v1.fill();
    cout << v1 << endl;
    Vec2f v2 = v1;
    cout << v2 << endl;
    v2.fill(0.1);
    cout << v2 << endl;
    cout << (v1 -= v2) << endl;
 
    Rotation2f rot1;
    cout << "created" << endl;
    cout << rot1 << endl;
    
    rot1.setIdentity();
    cout << "setIdentity" << endl;
    cout << rot1 << endl;
    
    cout << "getAngle" << endl;
    cout << rot1.getAngle();

    cout << "setAngle" << endl;
    rot1.setAngle(M_PI/2);
    cout << rot1 << endl;
    cout << rot1.getAngle() << endl;

    Rotation2f rot_acc;
    rot_acc.setIdentity();
    Rotation2f rot_inc;
    rot_inc.setAngle(M_PI/180);

    for (int i=0; i<360; ++i) {
        rot_acc = rot_acc * rot_inc;
        float angle_in_radians = rot_acc.getAngle();
        cout << "i: " << i << "alpha: " << angle_in_radians*180.f/M_PI << endl;
    }

    Rotation2f r_boh(0.5);
    cout << r_boh << endl << r_boh * r_boh.inverse() << endl;
    
    cout << "************************" << endl;
    Vec2f v_mult;
    v_mult.values[0] = 20;
    v_mult.values[1] == 5;
    cout << v_mult << endl;
    v_mult = r_boh * v_mult;
    cout << v_mult << endl;
    cout << r_boh.inverse()*v_mult << endl;

    Isometry2f iso(0.1, 0, 0.01); // move 10cm on the x-axis, 0cm on y-axis
    Isometry2f pose;
    pose.setIdentity();
    
    for (int k=0; k<1000; ++k) {
        for (int i=0; i<10000; ++i) {
            pose = pose * iso;
        }
        cout << ".";
    }

    return 0;
}
