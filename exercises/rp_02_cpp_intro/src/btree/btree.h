#pragma once
#include <iostream>

struct TreeNodeInt {
    // Fields
    int value;
    TreeNodeInt *left, *right;
    
    // Constructor
    TreeNodeInt(int value=0, TreeNodeInt* left=0, TreeNodeInt* right=0);
    
    // Destructor
    ~TreeNodeInt();

    // Methods
    TreeNodeInt* find(int value);  
    bool add(int value);
};

std::ostream& operator <<(std::ostream& os, const TreeNodeInt& node);
