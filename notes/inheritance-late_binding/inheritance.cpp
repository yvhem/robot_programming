#include <iostream>

using namespace std;

/* Classes (and structs) can be extended. */

struct A {
    // Fields
    int my_value;
    
    // Constructor
    A(int v) : my_value(v) { 
        cerr << "A [" << this << "] ctor" << endl;
    }

    // Destructor
    ~A() { 
        cerr << "A [" << this << "] dtor" << endl;
    }

    // Methods
    void print() {
        cerr << "A::print() [" << this << "]: my_value = " << my_value << endl;
    }
};

struct B: public A {
    // Fields
    int my_other_value;

    // Constructor
    B() : A(0), my_other_value(-1) { // basically my_value=0, my_other_value=-1
        cerr << "B [" << this << "] ctor" << endl;
    }

    // Destructor
    ~B() {
        cerr << "B [" << this << "] dtor" << endl;
    }

    // Methods
    void print() {
        A::print(); // super.print();
        cerr << "B::print() [" << this << "]: my_other_value = " 
            << my_other_value << endl;
    }
};

int main(int argc, char** argv) {
    cerr << "create A " << endl;
    A a(3);
    a.print();  // my_value = 3

    cerr << "create B " << endl;
    B b;
    b.print();  // my_value = 0, my_other_value = -1

    A a2 = b;   // Only common fields are copied
    a2.print(); // my_value = 0

    return 0;
}
