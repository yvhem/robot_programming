#pragma once
#include "vec_f.h"

struct MatF {
    int rows, cols, dimension;
    float* v;
    
    MatF();
    MatF(int rows, int cols);
    MatF(const MatF& other);
    ~MatF();

    MatF& operator=(const MatF& other);         // Deep copy
    MatF operator+(const MatF& other) const;    // this + other
    MatF operator-(const MatF& other) const;    // this - other
    MatF operator*(float f) const;              // f*this
    VecF operator*(const VecF& other) const;    // this * other
    MatF operator*(const MatF& other) const;    // this * other
    MatF transpose() const;                     // this^T
  
    void fill(float f);     // Fill the matrix with f
    void randFill();        // Fill the matrix with random values
    float& at(int i);
    const float& at(int i) const;
    float& at(int r, int c);
    const float& at(int r, int c) const;
};

std::ostream& operator <<(std::ostream& os, const MatF& v);
