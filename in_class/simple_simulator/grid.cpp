#include "grid.h"
#include "linalg.h"

// Constructor
Grid::Grid(int rows, int cols) { resize(rows, cols); }

// Destructor
Grid::~Grid() { clear(); }

// Methods
void Grid::clear() {
    if (!values) return;
    delete[] values;
    values = 0; rows = 0; cols = 0;
}

void Grid::resize(int r, int c) {
    if (r == rows && c == cols) return;
    clear();
    rows = r; cols = c; values = new CellType[rows*cols];
}

/* Travel unit by unit and return the length of the elapsed course.
 * Basically return the range from the current point to the obstacle. */
int Grid::scanSegment(int& x, int& y, float angle,
                      const CellType& val_min, const int max_range) const {
    Vec2f dir(cos(angle), sin(angle));  // Unit vector
    Vec2f current(x, y);                // Source vector
    int range = 0;
    while (range <= max_range) {
        // Check whether the current point is out of the bounds
        if (!inside(current.x(), current.y())) return -1;

        const CellType& target = at(current.x(), current.y());
        if (target < val_min) {         // If target is an obstacle
            x = current.x(); y = current.y();
            return range;               // Return the range
        }
        current += dir;                 // Travel
        ++range;                        // Increment the range up to now
    }
    return -1;
}
