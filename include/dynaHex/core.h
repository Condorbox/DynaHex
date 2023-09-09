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

        [[nodiscard]] Vector3 unit() const {
            Vector3 result = *this;
            result.normalize();
            return result;
        }
    };

    class Quaternion {
    public:
        union {
            struct {
                /**
                 * Holds the real component of the quaternion.
                 */
                real r;

                /**
                 * Holds the first complex component of the
                 * quaternion.
                 */
                real i;

                /**
                 * Holds the second complex component of the
                 * quaternion.
                 */
                real j;

                /**
                 * Holds the third complex component of the
                 * quaternion.
                 */
                real k;
            };

            /**
             * Holds the quaternion data in array form.
             */
            real data[4]{};
        };

        /**
         * The default constructor creates a quaternion representing
         * a zero rotation.
         */
        Quaternion() : r(1), i(0), j(0), k(0) {}

        /**
         * The explicit constructor creates a quaternion with the given
         * components.
         *
         * @param r The real component of the rigid body's orientation
         * quaternion.
         *
         * @param i The first complex component of the rigid body's
         * orientation quaternion.
         *
         * @param j The second complex component of the rigid body's
         * orientation quaternion.
         *
         * @param k The third complex component of the rigid body's
         * orientation quaternion.
         *
         * @note The given orientation does not need to be normalised,
         * and can be zero. This function will not alter the given
         * values, or normalise the quaternion. To normalise the
         * quaternion (and make a zero quaternion a legal rotation),
         * use the normalise function.
         *
         * @see normalise
         */
        Quaternion(const real r, const real i, const real j, const real k) : r(r), i(i), j(j), k(k) {}

        /**
         * Normalises the quaternion to unit length, making it a valid
         * orientation quaternion.
         */
        void normalise() {
            real d = r*r+i*i+j*j+k*k;

            // Check for zero length quaternion, and use the no-rotation
            // quaternion in that case.
            if (d < real_epsilon) {
                r = 1;
                return;
            }

            d = ((real)1.0)/real_sqrt(d);
            r *= d;
            i *= d;
            j *= d;
            k *= d;
        }

        /**
         * Multiplies the quaternion by the given quaternion.
         *
         * @param multiplier The quaternion by which to multiply.
         */
        void operator *=(const Quaternion &multiplier) {
            Quaternion q = *this;
            r = q.r*multiplier.r - q.i*multiplier.i -
                q.j*multiplier.j - q.k*multiplier.k;
            i = q.r*multiplier.i + q.i*multiplier.r +
                q.j*multiplier.k - q.k*multiplier.j;
            j = q.r*multiplier.j + q.j*multiplier.r +
                q.k*multiplier.i - q.i*multiplier.k;
            k = q.r*multiplier.k + q.k*multiplier.r +
                q.i*multiplier.j - q.j*multiplier.i;
        }

        /**
         * Adds the given vector to this, scaled by the given amount.
         * This is used to update the orientation quaternion by a rotation
         * and time.
         *
         * @param vector The vector to add.
         *
         * @param scale The amount of the vector to add.
         */
        void addScaledVector(const Vector3& vector, real scale) {
            Quaternion q(0, vector.x * scale, vector.y * scale, vector.z * scale);
            q *= *this;
            r += q.r * ((real)0.5);
            i += q.i * ((real)0.5);
            j += q.j * ((real)0.5);
            k += q.k * ((real)0.5);
        }

        void rotateByVector(const Vector3& vector) {
            Quaternion q(0, vector.x, vector.y, vector.z);
            (*this) *= q;
        }
    };

    class Matrix3 {
    public:
        // Holds the tensor matrix data in array form.
        real data[9];

        [[nodiscard]] Vector3 operator*(const Vector3 &vector) const {
            return Vector3 {
                    vector.x * data[0] + vector.y * data[1] + vector.z * data[2],
                    vector.x * data[3] + vector.y * data[4] + vector.z * data[5],
                    vector.x * data[6] + vector.y * data[7] + vector.z * data[8]
            };
        }

        [[nodiscard]] Matrix3 operator*(const Matrix3 &o) const {
            return Matrix3 {
                    data[0] * o.data[0] + data[1] * o.data[3] + data[2] * o.data[6],
                    data[0] * o.data[1] + data[1] * o.data[4] + data[2] * o.data[7],
                    data[0] * o.data[2] + data[1] * o.data[5] + data[2] * o.data[8],
                    data[3] * o.data[0] + data[4] * o.data[3] + data[5] * o.data[6],
                    data[3] * o.data[1] + data[4] * o.data[4] + data[5] * o.data[7],
                    data[3] * o.data[2] + data[4] * o.data[5] + data[5] * o.data[8],
                    data[6] * o.data[0] + data[7] * o.data[3] + data[8] * o.data[6],
                    data[6] * o.data[1] + data[7] * o.data[4] + data[8] * o.data[7],
                    data[6] * o.data[2] + data[7] * o.data[5] + data[8] * o.data[8]
            };
        }

        void operator*=(const Matrix3 &o) {
            real t1;
            real t2;
            real t3;
            t1 = data[0]*o.data[0] + data[1]*o.data[3] + data[2]*o.data[6];
            t2 = data[0]*o.data[1] + data[1]*o.data[4] + data[2]*o.data[7];
            t3 = data[0]*o.data[2] + data[1]*o.data[5] + data[2]*o.data[8];
            data[0] = t1;
            data[1] = t2;
            data[2] = t3;
            t1 = data[3]*o.data[0] + data[4]*o.data[3] + data[5]*o.data[6];
            t2 = data[3]*o.data[1] + data[4]*o.data[4] + data[5]*o.data[7];
            t3 = data[3]*o.data[2] + data[4]*o.data[5] + data[5]*o.data[8];
            data[3] = t1;
            data[4] = t2;
            data[5] = t3;
            t1 = data[6]*o.data[0] + data[7]*o.data[3] + data[8]*o.data[6];
            t2 = data[6]*o.data[1] + data[7]*o.data[4] + data[8]*o.data[7];
            t3 = data[6]*o.data[2] + data[7]*o.data[5] + data[8]*o.data[8];
            data[6] = t1;
            data[7] = t2;
            data[8] = t3;
        }

        // Transform the given vector by this matrix.
        [[nodiscard]] Vector3 transform(const Vector3 &vector) const {
            return (*this) * vector;
        }

        // Sets the matrix to be the inverse of the given matrix.
        void setInverse(const Matrix3 &m) {
            real t1 = m.data[0]*m.data[4];
            real t2 = m.data[0]*m.data[5];
            real t3 = m.data[1]*m.data[3];
            real t4 = m.data[2]*m.data[3];
            real t5 = m.data[1]*m.data[6];
            real t6 = m.data[2]*m.data[6];
            // Calculate the determinant.
            real det = (t1*m.data[8] - t2*m.data[7] - t3*m.data[8]+ t4*m.data[7] + t5*m.data[5] - t6*m.data[4]);
            // Make sure the determinant is non-zero.
            if (det == (real)0.0f) return;
            real invd = (real)1.0f/det;
            data[0] = (m.data[4]*m.data[8]-m.data[5]*m.data[7])*invd;
            data[1] = -(m.data[1]*m.data[8]-m.data[2]*m.data[7])*invd;
            data[2] = (m.data[1]*m.data[5]-m.data[2]*m.data[4])*invd;
            data[3] = -(m.data[3]*m.data[8]-m.data[5]*m.data[6])*invd;
            data[4] = (m.data[0]*m.data[8]-t6)*invd;
            data[5] = -(t2-t4)*invd;
            data[6] = (m.data[3]*m.data[7]-m.data[4]*m.data[6])*invd;
            data[7] = -(m.data[0]*m.data[7]-t5)*invd;
            data[8] = (t1-t3)*invd;
        }

        // Returns a new matrix containing the inverse of this matrix.
        [[nodiscard]] Matrix3 inverse() const {
            Matrix3 result{};
            result.setInverse(*this);
            return result;
        }

        // Inverts the matrix.
        void invert() {
            setInverse(*this);
        }

        // Sets the matrix to be the transpose of the given matrix.
        void setTranspose(const Matrix3 &m) {
            data[0] = m.data[0];
            data[1] = m.data[3];
            data[2] = m.data[6];
            data[3] = m.data[1];
            data[4] = m.data[4];
            data[5] = m.data[7];
            data[6] = m.data[2];
            data[7] = m.data[5];
            data[8] = m.data[8];
        }

        // Returns a new matrix containing the transpose of this matrix.
        [[nodiscard]] Matrix3 transpose() const {
            Matrix3 result{};
            result.setTranspose(*this);
            return result;
        }

        /**
         * Sets this matrix to be the rotation matrix corresponding to
         * the given quaternion.
         */
        void setOrientation(const Quaternion &q) {
            data[0] = 1 - (2*q.j*q.j + 2*q.k*q.k);
            data[1] = 2*q.i*q.j + 2*q.k*q.r;
            data[2] = 2*q.i*q.k - 2*q.j*q.r;
            data[3] = 2*q.i*q.j - 2*q.k*q.r;
            data[4] = 1 - (2*q.i*q.i  + 2*q.k*q.k);
            data[5] = 2*q.j*q.k + 2*q.i*q.r;
            data[6] = 2*q.i*q.k + 2*q.j*q.r;
            data[7] = 2*q.j*q.k - 2*q.i*q.r;
            data[8] = 1 - (2*q.i*q.i  + 2*q.j*q.j);
        }
        /**
         * Interpolates a couple of matrices.
         */
        static Matrix3 linearInterpolate(const Matrix3& a, const Matrix3& b, real prop);
    };

    class Matrix4 {
    public:
        // Holds the tensor matrix data in array form.
        // It is assumed that the remaining four are (0,0,0,1), producing a homogenous matrix.
        real data[12];

        [[nodiscard]] Vector3 operator*(const Vector3 &vector) const
        {
            return Vector3{
                    vector.x * data[0] +
                    vector.y * data[1] +
                    vector.z * data[2] +
                    data[3],

                    vector.x * data[4] +
                    vector.y * data[5] +
                    vector.z * data[6] +
                    data[7],

                    vector.x * data[8] +
                    vector.y * data[9] +
                    vector.z * data[10] +
                    data[11]
            };
        }

        [[nodiscard]] Matrix4 operator*(const Matrix4 &o) const {
            Matrix4 result{};

            result.data[0] = o.data[0]*data[0] + o.data[4]*data[1] + o.data[8]*data[2];
            result.data[4] = o.data[0]*data[4] + o.data[4]*data[5] + o.data[8]*data[6];
            result.data[8] = o.data[0]*data[8] + o.data[4]*data[9] + o.data[8]*data[10];
            result.data[1] = o.data[1]*data[0] + o.data[5]*data[1] + o.data[9]*data[2];
            result.data[5] = o.data[1]*data[4] + o.data[5]*data[5] + o.data[9]*data[6];
            result.data[9] = o.data[1]*data[8] + o.data[5]*data[9] + o.data[9]*data[10];
            result.data[2] = o.data[2]*data[0] + o.data[6]*data[1] + o.data[10]*data[2];
            result.data[6] = o.data[2]*data[4] + o.data[6]*data[5] + o.data[10]*data[6];
            result.data[10]= o.data[2]*data[8] + o.data[6]*data[9] + o.data[10]*data[10];
            result.data[3] = o.data[3]*data[0] + o.data[7]*data[1] + o.data[11]*data[2] + data[3];
            result.data[7] = o.data[3]*data[4] + o.data[7]*data[5] + o.data[11]*data[6] + data[7];
            result.data[11]= o.data[3]*data[8] + o.data[7]*data[9] + o.data[11]*data[10] + data[11];

            return result;
        }

        // Transform the given vector by this matrix.
        [[nodiscard]] Vector3 transform(const Vector3 &vector) const {
            return (*this) * vector;
        }

        [[nodiscard]] real getDeterminant() const;

        // Sets the matrix to be the inverse of the given matrix.
        void setInverse(const Matrix4 &m);

        // Returns a new matrix containing the inverse of this matrix.
        [[nodiscard]] Matrix4 inverse() const {
            Matrix4 result{};
            result.setInverse(*this);
            return result;
        }

        // Inverts the matrix.
        void invert() {
            setInverse(*this);
        }

        /**
        * Sets this matrix to be the rotation matrix corresponding to
        * the given quaternion.
        */
        void setOrientationAndPos(const Quaternion &q, const Vector3 &pos) {
            data[0] = 1 - (2*q.j*q.j + 2*q.k*q.k);
            data[1] = 2*q.i*q.j + 2*q.k*q.r;
            data[2] = 2*q.i*q.k - 2*q.j*q.r;
            data[3] = pos.x;
            data[4] = 2*q.i*q.j - 2*q.k*q.r;
            data[5] = 1 - (2*q.i*q.i + 2*q.k*q.k);
            data[6] = 2*q.j*q.k + 2*q.i*q.r;
            data[7] = pos.y;
            data[8] = 2*q.i*q.k + 2*q.j*q.r;
            data[9] = 2*q.j*q.k - 2*q.i*q.r;
            data[10] = 1 - (2*q.i*q.i + 2*q.j*q.j);
            data[11] = pos.z;
        }

        /**
        * Transform the given vector by the transformational inverse
        * of this matrix.
        */
        [[nodiscard]] Vector3 transformInverse(const Vector3 &vector) const {
            Vector3 tmp = vector;
            tmp.x -= data[3];
            tmp.y -= data[7];
            tmp.z -= data[11];
            return Vector3 {
                    tmp.x * data[0] +
                    tmp.y * data[4] +
                    tmp.z * data[8],
                    tmp.x * data[1] +
                    tmp.y * data[5] +
                    tmp.z * data[9],
                    tmp.x * data[2] +
                    tmp.y * data[6] +
                    tmp.z * data[10]
            };
        }

        /**
        * Transform the given direction vector by this matrix.
        */
        [[nodiscard]] Vector3 transformDirection(const Vector3 &vector) const {
            return Vector3 {
                    vector.x * data[0] +
                    vector.y * data[1] +
                    vector.z * data[2],
                    vector.x * data[4] +
                    vector.y * data[5] +
                    vector.z * data[6],
                    vector.x * data[8] +
                    vector.y * data[9] +
                    vector.z * data[10]
            };
        }

        /**
        * Transform the given direction vector by the
        * transformational inverse of this matrix.
        */
        [[nodiscard]] Vector3 transformInverseDirection(const Vector3 &vector) const {
            return Vector3{
                    vector.x * data[0] +
                    vector.y * data[4] +
                    vector.z * data[8],
                    vector.x * data[1] +
                    vector.y * data[5] +
                    vector.z * data[9],
                    vector.x * data[2] +
                    vector.y * data[6] +
                    vector.z * data[10]
            };
        }
    };
}

#endif