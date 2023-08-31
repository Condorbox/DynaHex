#ifndef DYNAHEX_PRECISION_H
#define DYNAHEX_PRECISION_H

#include <cfloat>

namespace dynahex {
    /** Defines a real number precision. dynahex can be compiled in
    * single- or double-precision versions. By default, single
    * precision is provided.*/
    typedef float real;

    /** Defines the highest value for the real number. */
    #define REAL_MAX FLT_MAX

    /** Defines the precision of the square root operator. */
    #define real_sqrt sqrtf

    /** Defines the precision of the power operator. */
    #define real_pow powf

    /** Defines the precision of the absolute magnitude operator. */
    #define real_abs fabsf

    /** Defines the precision of the sine operator. */
    #define real_sin sinf

    /** Defines the precision of the cosine operator. */
    #define real_cos cosf

    /** Defines the precision of the exponent operator. */
    #define real_exp expf
}

#endif