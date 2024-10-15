#pragma once
#include <iostream>

struct VecF {
    // Fields
    int dim;
    float* v;
    
    // Constructors and destructor
    VecF();                     // Default constructor
    VecF(int dim_);             // Parameterized constructor
    VecF(const VecF& other);    // Copy constructor
    ~VecF();                    // Destructor

    // Overload of operators
    VecF& operator=(const VecF& other);         // Deep copy
    VecF operator+(const VecF& other) const;    // this + other
    VecF operator-(const VecF& other) const;    // this - other
    VecF operator*(float f) const;              // f*this
    float operator*(const VecF& other) const;   // this * other (dot product)

    // Methods
    float& at(int i);               // Read/Write access to element at position i
    const float& at(int i) const;   // Read access to element at position i
};

std::ostream& operator<<(std::ostream& os, const VecF& v);
