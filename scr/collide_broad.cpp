#include "dynaHex/collide_broad.h"

using namespace dynahex;

BoundingSphere::BoundingSphere(const Vector3 &center, real radius) {
    BoundingSphere::center = center;
    BoundingSphere::radius = radius;
}

BoundingSphere::BoundingSphere(const BoundingSphere &one, const BoundingSphere &two) {
    Vector3 centerOffset = two.center - one.center;
    real distance = centerOffset.squareMagnitude();
    real radiusDiff = two.radius - one.radius;

    // Check if the larger sphere encloses the small one
    if (radiusDiff*radiusDiff >= distance) {
        if (one.radius > two.radius) {
            center = one.center;
            radius = one.radius;
        }
        else {
            center = two.center;
            radius = two.radius;
        }
    }

    // Otherwise we need to work with partially overlapping spheres
    else {
        distance = real_sqrt(distance);
        radius = (distance + one.radius + two.radius) * ((real)0.5);

        // The new centre is based on one's centre, moved towards
        // two's centre by an amount proportional to the spheres'
        // radii.
        center = one.center;
        if (distance > 0) {
            center += centerOffset * ((radius - one.radius) / distance);
        }
    }
}

bool BoundingSphere::overlaps(const BoundingSphere *other) const {
    real distanceSquared = (center - other->center).squareMagnitude();
    return distanceSquared < (radius+other->radius)*(radius+other->radius);
}
