#include "btree.h"

TreeNodeInt::TreeNodeInt(int value_, TreeNodeInt* left_, TreeNodeInt* right_):
    value(value_), left(left_), right(right_) {}

TreeNodeInt::~TreeNodeInt() {
    if (left) delete left;
    if (right) delete right;
}

TreeNodeInt* TreeNodeInt::find(int value_) {
    if (value == value_) return this;
    if (value_ < value) {
        if (!left) return nullptr;
        return left->find(value_);
    }
    if (value_>value) {
        if (! right) return nullptr;
        return right->find(value_);
    }
    return nullptr;
}
  
bool TreeNodeInt::add(int value_) {
    if (value == value_) return false;
    if (value_ < value) {
        if (!left) {
            left = new TreeNodeInt(value_);
            return true;
        }
        return left->add(value_);
    }
    if (value_ > value) {
        if (!right) {
            right = new TreeNodeInt(value_);
            return true;
        }
        return right->add(value_);
    }
    return false;
}

std::ostream& operator <<(std::ostream& os, const TreeNodeInt& node) {
    if (node.left) os << *(node.left);
    os << node.value << " ";
    if (node.right) os << *(node.right);
    return os;
}
