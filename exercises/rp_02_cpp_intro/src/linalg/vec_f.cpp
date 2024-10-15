#include <iostream>
#include <assert.h>
#include "vec_f.h"

using namespace std;

// Constructors and destructor
VecF::VecF(): dim(0), v(nullptr) {}

VecF::VecF(int dim_): dim(dim_), v(nullptr) {
    if (dim) v = new float [dim];
}

VecF::VecF(const VecF& other): VecF(other.dim) {
    for (int i=0; i<dim; ++i) v[i]=other.v[i];
}

VecF::~VecF() { if(dim) delete[] v; }

// Overload of operators
VecF& VecF::operator=(const VecF& other) {
    // Handle self-assignment
    if (this == &other) return *this;

    // Rewrite the memory
    delete[] v;
    dim = other.dim; v = new float[dim];
    for (int i=0; i < dim; ++i) v[i] = other.v[i];
    
    return *this;
}

VecF VecF::operator+(const VecF& other) const {
    assert(other.dim == dim && "dim mismatch");
    VecF result(*this);
    for (int i=0; i < dim; ++i) result.v[i] += other.v[i];
    return result;
}

VecF VecF::operator-(const VecF& other) const {
    assert(other.dim == dim && "dim mismatch");
    VecF result(*this);
    for (int i=0; i < dim; ++i) result.v[i] -= other.v[i];
    return result;
}

VecF VecF::operator*(float f) const {
    VecF result(*this);
    for (int i=0; i < dim; ++i) result.v[i] *= f;
    return result;
}

float VecF::operator*(const VecF& other) const {
    assert(dim == other.dim);
    float result = 0.f;
    for (int i=0; i < dim; ++i) result += v[i] * other.v[i];
    return result;
}

// Methods
float& VecF::at(int i){ return v[i]; }
const float& VecF::at(int i) const { return v[i]; }

ostream& operator<<(ostream& os, const VecF& v) {
    os << "{ ptr: " << &v << " dim:" << v.dim << " vals: [ ";
    for (int i=0; i<v.dim; ++i)
        os << v.at(i) <<  " ";
    os << "] }";
    return os;
}
