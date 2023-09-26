
#ifndef DYNAHEX_COLLIDE_NARROW_H
#define DYNAHEX_COLLIDE_NARROW_H

#include "contacts.h"
namespace dynahex {
    // Forward declarations of primitive friends
    class IntersectionTests;
    class CollisionDetector;

    /**
     * Represents a primitive to detect collisions against.
     */
    class CollisionPrimitive
    {
    public:
        /**
         * This class exists to help the collision detector
         * and intersection routines, so they should have
         * access to its data.
         */
        friend class IntersectionTests;
        friend class CollisionDetector;
        /**
         * The rigid body that is represented by this primitive.
         */
        RigidBody * body;
        /**
         * The offset of this primitive from the given rigid body.
         */
        Matrix4 offset;
        /**
         * Calculates the internals for the primitive.
         */
        void calculateInternals();
        /**
         * This is a convenience function to allow access to the
         * axis vectors in the transform for this primitive.
         */
        [[nodiscard]] Vector3 getAxis(unsigned index) const {
            return transform.getAxisVector(index);
        }
        /**
         * Returns the resultant transform of the primitive, calculated from
         * the combined offset of the primitive and the transform
         * (orientation + position) of the rigid body to which it is
         * attached.
         */
        [[nodiscard]] const Matrix4& getTransform() const {
            return transform;
        }
    protected:
        /**
         * The resultant transform of the primitive. This is
         * calculated by combining the offset of the primitive
         * with the transform of the rigid body.
         */
        Matrix4 transform;
    };
    /**
     * Represents a rigid body that can be treated as a sphere
     * for collision detection.
     */
    class CollisionSphere : public CollisionPrimitive
    {
    public:
        /**
         * The radius of the sphere.
         */
        real radius;
    };

    /**
     * The plane is not a primitive: it doesn't represent another
     * rigid body. It is used for contacts with the immovable
     * world geometry.
     */
    class CollisionPlane : public CollisionPrimitive
    {
    public:
        /**
         * The plane normal
         */
        Vector3 direction;

        /**
         * The distance of the plane from the origin.
         */
        real offset;
    };
    /**
     * A helper structure that contains information for the detector to use
     * in building its contact data.
     */
    struct CollisionData
    {
        /**
         * Holds the base of the collision data: the first contact
         * in the array. This is used so that the contact pointer (below)
         * can be incremented each time a contact is detected, while
         * this pointer points to the first contact found.
         */
        Contact *contactArray;
        /** Holds the contact array to write into. */
        Contact *contacts;
        /** Holds the maximum number of contacts the array can take. */
        int contactsLeft;
        /** Holds the number of contacts found so far. */
        unsigned contactCount;
        /** Holds the friction value to write into any collisions. */
        real friction;
        /** Holds the restitution value to write into any collisions. */
        real restitution;
        /**
         * Holds the collision tolerance, even uncolliding objects this
         * close should have collisions generated.
         */
        real tolerance;
        /**
         * Checks if there are more contacts available in the contact
         * data.
         */
        bool hasMoreContacts() {
            return contactsLeft > 0;
        }

        /**
         * Resets the data so that it has no used contacts recorded.
         */
        void reset(unsigned maxContacts) {
            contactsLeft = maxContacts;
            contactCount = 0;
            contacts = contactArray;
        }
        /**
         * Notifies the data that the given number of contacts have
         * been added.
         */
        void addContacts(unsigned count)
        {
            // Reduce the number of contacts remaining, add number used
            contactsLeft -= count;
            contactCount += count;

            // Move the array forward
            contacts += count;
        }
    };
}

#endif //DYNAHEX_COLLIDE_NARROW_H
