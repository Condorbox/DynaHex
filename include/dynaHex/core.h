#include <cmath>

#ifndef DYNAHEX_CORE_H
#define DYNAHEX_CORE_H

#include "precision.h"

namespace dynahex {
    class Vector3 {
    public:
        real x;
        real y;
        real z;

    private:
        /** Padding to ensure 4 word alignment.  we can remove it safely*/
        real pad;

    public:

        Vector3() : x(0), y(0), z(0), pad(0) {}

        Vector3(const real x, const real y, const real z) : x(x), y(y), z(z), pad(0) {}

        const static Vector3 GRAVITY;
        const static Vector3 HIGH_GRAVITY;
        const static Vector3 UP;
        const static Vector3 RIGHT;
        const static Vector3 OUT_OF_SCREEN;
        const static Vector3 X;
        const static Vector3 Y;
        const static Vector3 Z;
        const static Vector3 Zero;

        real operator[](unsigned i) const {
            if (i == 0) return x;
            else if (i == 1) return y;
            return z;
        }

        real &operator[](unsigned i) {
            if (i == 0) return x;
            else if (i == 1) return y;
            return z;
        }

        Vector3 operator+(const Vector3 &v) const {
            return {x + v.x, y + v.y, z + v.z}; //Braced initializer
        }

        Vector3 operator-(const Vector3 &v) const {
            return {x - v.x, y - v.y, z - v.z};
        }

        Vector3 operator*(const real value) const {
            return {x * value, y * value, z * value};
        }

        Vector3 operator/(const real value) const {
            if (value == (real)0.0) return {0, 0, 0};
            return {x / value, y / value, z * value};
        }

        //Calculates and returns the scalar product
        real operator*(const Vector3 &vector) const {
            return x * vector.x + y * vector.y + z * vector.z;
        }

        //Calculates and returns the vector product of this vector with the given vector
        Vector3 operator%(const Vector3 &vector) const {
            return {y * vector.z - z * vector.y, z * vector.x - x * vector.z, x * vector.y - y * vector.x};
        }

        void operator+=(const Vector3 &v) {
            x += v.x;
            y += v.y;
            z += v.z;
        }

        void operator-=(const Vector3 &v) {
            x -= v.x;
            y -= v.y;
            z -= v.z;
        }

        void operator*=(const real value) {
            x *= value;
            y *= value;
            z *= value;
        }

        // Updates this vector to be the vector product of its current value and the given vector
        void operator%=(const Vector3 &vector) {
            *this = vectorProduct(vector);
        }

        void invert() {
            x = -x;
            y = -y;
            z = -z;
        }

        [[nodiscard]] real magnitude() const {
            return real_sqrt(x * x + y * y + z * z);
        }

        [[nodiscard]] real squareMagnitude() const {
            return x * x + y * y + z * z;
        }

        void normalize() {
            real l = magnitude();
            if (l > 0) {
                (*this) *= ((real) 1) / l;
            }
        }

        void addScaledVector(const Vector3 &vector, const real scale) {
            x += vector.x * scale;
            y += vector.y * scale;
            z += vector.z * scale;
        }

        [[nodiscard]] Vector3 componentProduct(const Vector3 &vector) const {
            return {x * vector.x, y * vector.y, z * vector.z};
        }

        void componentProductUpdate(const Vector3 &vector) {
            x *= vector.x;
            y *= vector.y;
            z *= vector.z;
        }

        [[nodiscard]] real scalarProduct(const Vector3 &vector) const {
            return x * vector.x + y * vector.y + z * vector.z;
        }

        [[nodiscard]] Vector3 vectorProduct(const Vector3 &vector) const {
            return {y * vector.z - z * vector.y, z * vector.x - x * vector.z, x * vector.y - y * vector.x};
        }

        void clear() {
            x = y = z = 0;
        }
    };
}

#endif