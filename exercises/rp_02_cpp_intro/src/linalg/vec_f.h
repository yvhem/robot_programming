#pragma once
#include <iostream>

struct VecF {
    // Fields
    int dim;
    float* v;
    
    // Constructors
    VecF();                     // Default constructor
    VecF(int dim_);             // Constructor with dim
    VecF(const VecF& other);    // Copy constructor

    // Destructor
    ~VecF();

    // Methods

    // read/write access to element at pos
    float& at(int pos);
  
  // read access to element at pos
  // const after () means that the method does not modify the invoking object
  const float& at(int pos) const;

  // op =, deep copy
  VecF& operator =(const VecF& other);

  // returns the sum this + other
  VecF operator +(const VecF& other) const;

  // returns the difference this - other
  VecF operator -(const VecF& other) const;

  // returns this * f
  VecF  operator *(float f) const;

  // returns the dot product (vecs should have the same size);
  float operator *( const VecF& other) const;
};

std::ostream& operator <<(std::ostream& os, const VecF& v);
