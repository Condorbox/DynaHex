
#ifndef DYNAHEX_CONTACTS_H
#define DYNAHEX_CONTACTS_H

#include "body.h"
namespace dynahex {
    class Contact {
    public:
        /**
         * Holds the bodies that are involved in the contact. The
         * second of these can be NULL, for contacts with the scenery.
         */
        RigidBody* body[2];
        /**
         * Holds the lateral friction coefficient at the contact.
         */
        real friction;
        /**
         * Holds the normal restitution coefficient at the contact.
         */
        real restitution;
        /**
         * Holds the position of the contact in world coordinates.
         */
        Vector3 contactPoint;
        /**
         * Holds the direction of the contact in world coordinates.
         */
        Vector3 contactNormal;
        /**
         * Holds the depth of penetration at the contact point. If both
         * bodies are specified then the contact point should be midway
         * between the inter-penetrating points.
         */
        real penetration;
        /**
         * Sets the data that doesn't normally depend on the position
         * of the contact (i.e. the bodies, and their material properties).
         */
        void setBodyData(RigidBody* one, RigidBody *two, real friction, real restitution);
    };
}

#endif //DYNAHEX_CONTACTS_H
