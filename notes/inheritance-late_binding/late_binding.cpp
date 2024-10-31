#include <iostream>

using namespace std;

/* Inheritance. */
class A {
public:
    A(int value_) : _value(value_) {}

    void print() {
        cerr << "A: [" << this << "], value: " << _value << endl;
    }
protected:
    int _value;
};

class B: public A {
public:
    B(int value_) : A(value_) {}

    void print() {
        cerr << "B: [" << this << "], value: " << _value << endl;
    }
};
/* ############ */

/* Inheritance + Late Binding. */
class A_lb {
public:
    A_lb(int value_) : _value(value_) {}

    virtual void print() {
        cerr << "A_lb: [" << this << "], value: " << _value << endl;
    }
protected:
    int _value;
};

class B_lb: public A_lb {
public:
    B_lb(int value_) : A_lb(value_) {}

    void print() override {
        cerr << "B_lb: [" << this << "], value: " << _value << endl;
    }
};
/* ########################### */

int main(int argc, char** argv) {
    cerr << "##### NO LATE BINDING #####" << endl;
    {
        B b(10);
        A& a_ref = b;
        b.print();
        a_ref.print();
    }

    cerr << endl << "##### LATE BINDING #####" << endl;
    {
        B_lb b(10);
        A_lb& a_ref(b);
        b.print();
        a_ref.print();
        
        // a is a copy of b
        A_lb copy_of_b_as_a(b);
        copy_of_b_as_a.print();
    }
}
