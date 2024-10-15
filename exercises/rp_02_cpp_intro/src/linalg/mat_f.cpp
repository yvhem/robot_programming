#include "mat_f.h"
#include <assert.h>
#include <iostream>

using namespace std;

MatF::MatF(): rows(0), cols(0), dimension(0), v(nullptr) {}

MatF::MatF(int rows_, int cols_): 
    rows(rows_), cols(cols_), dimension(rows*cols), v(new float[dimension]) {}

MatF::MatF(const MatF& other): MatF(other.rows, other.cols) {
    for (int i=0; i < dimension; ++i) v[i] = other.v[i];
}

MatF::~MatF() { if (dimension) delete[] v; }

MatF& MatF::operator=(const MatF& other) {
    // Handle self-assignment
    if (this == &other) return *this;

    // Rewrite the memory
    delete[] v;
    rows = other.rows; cols = other.cols; dimension = other.dimension;
    v = new float[dimension];
    for (int i=0; i < dimension; ++i) v[i] = other.v[i];

    return *this;
}

MatF MatF::operator+(const MatF& other) const {
    assert(other.cols == cols && other.rows == rows && "dim mismatch");
    MatF result(*this);
    for (int i=0; i < dimension; ++i) result.v[i] += other.v[i];
    return result;
}

MatF MatF::operator-(const MatF& other) const {
    assert(other.cols == cols && other.rows == rows && "dim mismatch");
    MatF result(*this);
    for (int i=0; i < dimension; ++i) result.v[i] -= other.v[i];
    return result;
}  

MatF MatF::operator*(float f) const {
    MatF result(*this);
    for (int i=0; i < dimension; ++i) result.v[i] *= f;
    return result;
}

VecF MatF::operator*(const VecF& other) const {
    assert(other.dim == cols && "dim mismatch");
    VecF result(rows);
    for (int r=0; r < rows; ++r) {
        float accumulator = 0.f;
        for (int c=0; c < cols; ++c) accumulator += at(r,c) * other.at(c);
        result.at(r) = accumulator;
    }
    return result;
}

MatF MatF::operator*(const MatF& other) const {
    assert(cols == other.rows && "dimension mismatch");
    MatF result(rows, other.cols);
    for (int dc = 0; dc < other.cols; ++dc) {
        for (int r=0; r < rows; ++r) {
            float& dest = result.at(r, dc);
            dest = 0.f;
            for (int c=0; c < cols; ++c) dest += at(r,c) * other.at(c,dc);
        }
    }
    return result;
}

MatF MatF::transpose() const {
    MatF returned(cols, rows);
    for (int r=0; r < rows; ++r)
        for (int c=0; c < cols; ++c)
            returned.at(c,r) = at(r,c);
    return returned;
}

void MatF::fill(float f) {
    for (int i=0; i<dimension; ++i) v[i] = f;
}

void MatF::randFill() {
    for (int i=0; i<dimension; ++i) v[i] = drand48();
}

float& MatF::at(int i) { return v[i]; }  
const float& MatF::at(int i) const { return v[i]; }
float& MatF::at(int r, int c) { return v[r*cols+c]; }
const float& MatF::at(int r, int c) const { return v[r*cols+c]; }

std::ostream& operator <<(std::ostream& os, const MatF& m) {
    os << "{ ptr: " << &m <<", rows: " << m.rows << ",  cols: " << m.cols << endl;
    os << "  values:[ " << endl;
    for (int r=0; r<m.rows; ++r) {
        os << "\t";
        for (int c=0; c<m.cols; ++c) os << m.at(r,c) <<  " ";
        os << endl;
    }
    os << " ]" << endl << "}";
    return os;
}
