
#ifndef DYNAHEX_FGEN_H
#define DYNAHEX_FGEN_H

#include "body.h"

namespace dynahex {
    class ForceGenerator {
        /**
        * Overload this in implementations of the interface to calculate
        * and update the force applied to the given rigid body.
        */
        virtual void updateForce(RigidBody *body, real duration) = 0;
    };

    /**
    * A force generator that applies a gravitational force. One instance
    * can be used for multiple rigid bodies.
    */
    class Gravity : public ForceGenerator {
        /** Holds the acceleration due to gravity. */
        Vector3 gravity;
    public:
        /** Creates the generator with the given acceleration. */
        explicit Gravity(const Vector3 &gravity);
        /** Applies the gravitational force to the given rigid body. */
        void updateForce(RigidBody *body, real duration) override;
    };
    /**
     * A force generator that applies a Spring force.
     */
    class Spring : public ForceGenerator {
        /**
         * The point of connection of the spring, in local
         * coordinates.
         */
        Vector3 connectionPoint;
        /**
         * The point of connection of the spring to the other object,
         * in that object's local coordinates.
         */
        Vector3 otherConnectionPoint;
        /** The particle at the other end of the spring. */
        RigidBody *other;
        /** Holds the sprint constant. */
        real springConstant;
        /** Holds the rest length of the spring. */
        real restLength;
    public:
        /** Creates a new spring with the given parameters. */
        Spring(const Vector3 &localConnectionPt, RigidBody *other, const Vector3 &otherConnectionPt,
               real springConstant, real restLength);

        /** Applies the spring force to the given rigid body. */
        void updateForce(RigidBody *body, real duration) override;
    };

    class AngularVelocityController : public ForceGenerator {
        Vector3 targetAngularVelocity;
        real maxDistance;
    public:
        AngularVelocityController(const Vector3& targetAngularVelocity, real maxDistance);

        void updateForce(RigidBody *body, real duration) override;
    };
}


#endif //DYNAHEX_FGEN_H
