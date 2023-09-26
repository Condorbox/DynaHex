
#include "dynaHex/collide_narrow.h"

using namespace dynahex;

void CollisionPrimitive::calculateInternals() {
    transform = body->getTransform() * offset;
}
