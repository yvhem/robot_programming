#pragma once

using Scalar = float;
using namespace std;

struct Vec2f {
    // Fields
    static constexpr int Dim = 2;   /* constexpr = evaluated at compile-time */
    Scalar values[Dim];

    // Constructors
    Vec2f() {}
    Vec2f(const Scalar& x, const Scalar& y) {
        values[0] = x; values[1] = y;
    }

    // Destructor
    
    // Operator overload
    inline Vec2f& operator+=(const Vec2f& src) {
        for (int i=0; i<Dim; ++i) values[i] += src.values[i];
        return *this;
    }

    inline Vec2f& operator-=(const Vec2f& src) {
        for (int i=0; i<Dim; ++i) values[i] -= src.values[i];
        return *this;
    }

    inline Vec2f operator+(const Vec2f& other) const {
        Vec2f v = *this;
        v += other;
        return v;
    }
    
    inline Vec2f operator-(const Vec2f& other) const {
        Vec2f v = *this;
        v -= other;
        return v;
    }

    inline Vec2f operator-() const {
        return Vec2f(-values[0], -values[1]);
    }

    // Methods
    inline Scalar dot(const Vec2f& src) const {
        Scalar accumulator = Scalar(0);
        for (int i=0; i<Dim; ++i) accumulator += values[i] * src.values[i];
        return accumulator;
    }

    inline void fill(const Scalar v=Scalar(0)) {
        for (int i=0; i<Dim; ++i) values[i] = v;
    }

    Scalar norm() const {
        return sqrt(dot(*this));
    }
};

ostream& operator<<(ostream& os, const Vec2f& src) {
    os << "V2 [ " << &src << "] [";
    for (int i=0; i<Vec2f::Dim; ++i) os << src.values[i] << " ";
    os << "]";
    return os;
}

struct Rotation2f {
    // Fields
    static constexpr int Dim = 2;
    Scalar R[Dim][Dim];

    // Constructors
    Rotation2f() {}
    Rotation2f(const Scalar angle) {
        setAngle(angle);
    }

    // Operator overload
    inline Rotation2f operator*(const Rotation2f& other) const {
        Rotation2f res;
        for (int r=0; r<Dim; ++r)
            for (int c=0; c<Dim; ++c)
                res.R[r][c] = 0;
        for (int r=0; r<Dim; ++r)
            for (int c=0; c<Dim; ++c)
                for (int k=0; k<Dim; ++k)
                    res.R[r][c] += R[r][k] * other.R[k][c];
        return res;
    }

    Vec2f operator*(const Vec2f& src) {
        Vec2f res;
        res.fill(0.f);
        for (int r=0; r<Dim; ++r)
            for (int c=0; c<Dim; ++c)
                res.values[r] += R[r][c] * src.values[c];
        return res;
    }
    
    /*
    Rotation2f operator-=(const Rotation2f& src) {
        Rotation2f res;
        for(int r=0; r<Dim; ++r)
            for (int c=0; c<Dim; ++c)
                res.R[r][c] = -res.R[r][c]
        return res;
    }*/
    
    // Methods
    inline void setIdentity() {
        for (int r=0; r < Dim; ++r)
            for (int c=0; c<Dim; ++c)
                R[r][c] = (Scalar)(r == c);
    }

    Rotation2f inverse() const {
        Rotation2f res = *this;
        res.transposeInPlace();
        return res;
    }

    void inverseInPlace();

    const Rotation2f& transposeInPlace() {
        for (int r=0; r<Dim; ++r) {
            for (int c=0; c<Dim; ++c) {
                Scalar aux = R[r][c];
                R[r][c] = R[c][r];
                R[c][r] = aux;
            }
        }
        return *this;
    }

    Scalar getAngle() const {
        return atan2(R[1][0], R[0][0]);
    }
    
    void setAngle(Scalar angle) {
        Scalar s = sin(angle), c = cos(angle);
        R[0][0] = c; R[0][1] = -s;
        R[1][0] = s; R[1][1] = c;
    }
};

ostream& operator<<(ostream& os, const Rotation2f& src) {
    os << "R2 [" << &src<<"] [";
    for (int r=0; r<Rotation2f::Dim; ++r) {
        for (int c=0; c<Rotation2f::Dim; ++c) {
            os << src.R[r][c] << " ";
        }
    }
    return os;
}

struct Isometry2f {
    // Fields
    Vec2f t;
    Rotation2f R;

    // Constructors
    Isometry2f() {}
    Isometry2f(Scalar x, Scalar y, Scalar theta) {

    }
    Isometry2f(const Vec2f& translation, const Rotation2f& rotation):
        t(translation), R(rotation) {}

    // Destructor

    // Operator overload
    Isometry2f& operator*=(const Isometry2f& src) {
        return *this;
    }

    inline Isometry2f operator*(const Isometry2f& src) {
        Isometry2f res;
        res.R = R * src.R;
        res.t = R*src.t + t;
        return res;
    }

    Vec2f operator*(const Vec2f& src) {
        return R*src + t;
    }

    /*Isometry2f& operator-(const Isometry2f& src) const {
        return *this;
    }*/

    // Methods
    inline void setIdentity() {
        t.fill(0);
        R.setIdentity();
    }

    inline Isometry2f inverse() const {
        Isometry2f res;
        res.R = R.inverse();
        res.t = -(res.R*t);
        return res;
    }
};
