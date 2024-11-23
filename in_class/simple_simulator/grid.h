#pragma once
#include "linalg.h"
#include "grid.h"
#include "world.h"
#include "opencv2/opencv.hpp"
#include "canvas.h"

struct Grid {
    using CellType = uint8_t;
    
    // Fields
    int rows = 0;
    int cols = 0;
    CellType* values = nullptr;

    // Constructor
    Grid(int rows=0, int cols=0);

    // Destructor
    ~Grid();

    // Operator overload
    // Disable assignment and copy constructor
    Grid& operator=(const Grid&) = delete;
    Grid(const Grid&) = delete;

    // Methods
    inline CellType& at(int r, int c) {
        return values[r*cols + c];
    }

    inline const CellType& at(int r, int c) const {
        return values[r*cols + c];
    }

    inline bool inside(int r, int c) const {
        return (r >= 0) && (r < rows) && (c >= 0) && (c < cols);
    }

    inline std::pair<int, int> ptr2rc(const CellType* ptr) const {
        int offset = ptr - values;
        return std::pair<int, int>(offset/cols, offset%cols);
    }

    void clear();
    void resize(int r, int c);
    int scanSegment(int& x, int& y, float angle,
                    const CellType& val_min, const int max_range) const;
};
