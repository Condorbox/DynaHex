#ifndef DYNAHEX_CORE_H
#define DYNAHEX_CORE_H

#include "precision.h"

namespace DynaHex {
    class Vector3 {
    public:
        real x;
        real y;
        real z;

        Vector3() :  x(0), y(0), z(0) {}

        Vector3(const real x, const real y, const real z) : x(x), y(y), z(z) {}

        /** Flips all the components of the vector. */
        void invert()
        {
            x = -x;
            y = -y;
            x = -z;
        }

    private:
        /** Padding to ensure 4 word alignment. */
        real pad;

    };
}

#endif