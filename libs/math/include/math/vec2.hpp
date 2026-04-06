// Header file for the Vec2 class
//
#pragma once
#include <cmath> //Math functions

struct Vec2 {

    double x;
    double z;
    Vec2(double x, double z) : x(x), z(z) {}
    // + Overloading - Vec2 + Vec2 (summing their respective x and z components)
    // const means "this method doesn't modify the object it's called on."
    Vec2 operator+(const Vec2 &other) const {
        return Vec2(x + other.x, z + other.z);
    }
    Vec2 operator-(const Vec2 &other) const {
        return Vec2(x - other.x, z - other.z);
    }
    // * Overloading - Vec2 * double (scaling each component by the double)
    Vec2 operator*(double other) const {
        return Vec2(x * other, z * other);
    }
    // Dot product
    double dot(const Vec2 &other) const {
        return x * other.x + z * other.z;
    }
    double norm() const {
        return std::sqrt(x * x + z * z);
    }
    Vec2 normalized() const {
        double len = norm();
        if (x == 0 && z == 0) {
            return Vec2(0, 0);
        }
        return Vec2(x/len, z/len);
    }

};
