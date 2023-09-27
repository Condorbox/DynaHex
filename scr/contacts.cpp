
#include "dynaHex/contacts.h"

using namespace dynahex;

void Contact::setBodyData(RigidBody *one, RigidBody *two, real friction, real restitution) {
    Contact::body[0] = one;
    Contact::body[1] = two;
    Contact::friction = friction;
    Contact::restitution = restitution;
}
