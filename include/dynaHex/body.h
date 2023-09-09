
#ifndef DYNAHEX_BODY_H
#define DYNAHEX_BODY_H
#include "core.h"

namespace dynahex {
    /**
    * A rigid body is the basic simulation object in the physics
    * core.
    */
    class RigidBody {
    protected:
        /**
        * Holds the inverse of the mass of the rigid body. It
        * is more useful to hold the inverse mass because
        * integration is simpler, and because in real-time
        * simulation it is more useful to have bodies with
        * infinite mass (immovable) than zero mass
        * (completely unstable in numerical simulation).
        */
        real inverseMass;
        /**
        * Holds the amount of damping applied to linear
        * motion. Damping is required to remove energy added
        * through numerical instability in the integrator.
        */
        real linearDamping;
        /**
        * Holds the linear position of the rigid body in
        * world space.
        */
        Vector3 position;
        /**
        * Holds the angular orientation of the rigid body in
        * world space.
        */
        Quaternion orientation;
        /**
        * Holds the linear velocity of the rigid body in
        * world space.
        */
        Vector3 velocity;
        /**
        * Holds the angular velocity, or rotation, or the
        * rigid body in world space.
        */
        Vector3 rotation;
        /**
         * A body can be put to sleep to avoid it being updated
         * by the integration functions or affected by collisions
         * with the world.
         */
        bool isAwake;
        /**
        * Holds a transform matrix for converting body space into
        * world space and vice versa. This can be achieved by calling
        * the getPointIn*Space functions.
        */
        Matrix4 transformMatrix;
        /**
        * Holds the inverse of the body’s inertia tensor. The
        * inertia tensor provided must not be degenerate
        * (that would mean the body had zero inertia for
        * spinning along one axis). As long as the tensor is
        * finite, it will be invertible. The inverse tensor
        * is used for similar reasons to the use of inverse
        * mass.
        *
        * The inertia tensor, unlike the other variables that
        * define a rigid body, is given in body space.
        */
        Matrix3 inverseInertiaTensor;
        /**
         * Holds the inverse inertia tensor of the body in world
         * space. The inverse inertia tensor member is specified in
         * the body's local space.
         */
        Matrix3 inverseInertiaTensorWorld;
        /**
        * Holds the accumulated force to be applied at the next
        * integration step.
        */
        Vector3 forceAccum;
        /**
         * Holds the accumulated torque to be applied at the next
         * integration step.
         */
        Vector3 torqueAccum;
        /**
        * Holds the amount of damping applied to angular
        * motion. Damping is required to remove energy added
        * through numerical instability in the integrator.
        */
        real angularDamping;
        /**
          * Holds the acceleration of the rigid body.  This value
          * can be used to set acceleration due to gravity (its primary
          * use), or any other constant acceleration.
          */
        Vector3 acceleration;
        /**
         * Holds the linear acceleration of the rigid body, for the
         * previous frame.
         */
        Vector3 lastFrameAcceleration;
    public:
        /**
        * Calculates internal data from state data. This should be called
        * after the body’s state is altered directly (it is called
        * * automatically during integration). If you change the body’s state
        * and then intend to integrate before querying any data (such as
        * the transform matrix), then you can omit this step.
        */
        void calculateDerivedData();
        /**
         * Integrates the rigid body forward in time by the given amount.
         * This function uses a Newton-Euler integration method, which is a
         * linear approximation to the correct integral. For this reason it
         * may be inaccurate in some cases.
         */
        void integrate(real duration);

        void setInertiaTensor(const Matrix3 &inertiaTensor);

        /**
        * Adds the given force to center of mass of the rigid body.
        * The force is expressed in world coordinates.
        */
        void addForce(const Vector3 &force);
        /**
        * Clears the forces and torques in the accumulators. This will
        * be called automatically after each intergration step.
        */
        void clearAccumulators();
        /**
        * Adds the given force to the given point on the rigid body.
        * Both the force and the application point are given in world
        * space. Because the force is not applied at the center of
        * mass, it may be split into both a force and torque.
        */
        void addForceAtPoint(const Vector3 &force, const Vector3 &point);
        /**
        * Adds the given force to the given point on the rigid body.
        * The direction of the force is given in world coordinates,
        * but the application point is given in body space. This is
        * useful for spring forces, or other forces fixed to the
        * body.
        */
        void addForceAtBodyPoint(const Vector3 &force, const Vector3 &point);
        /**
         * Converts the given point from world space into the body's
         * local space.
         */
        [[nodiscard]] Vector3 getPointInWorldSpace(const Vector3 &point) const;
        /**
         * Returns true if the mass of the body is not-infinite.
         */
        [[nodiscard]] bool hasFiniteMass() const;
        /**
         * Sets the mass of the rigid body.
         *
         * @param mass The new mass of the body. This may not be zero.
         * Small masses can produce unstable rigid bodies under
         * simulation.
         *
         * @warning This invalidates internal data for the rigid body.
         * Either an integration function, or the calculateInternals
         * function should be called before trying to get any settings
         * from the rigid body.
         */
        void setMass(real mass);

        /**
         * Gets the mass of the rigid body.
         *
         * @return The current mass of the rigid body.
         */
        [[nodiscard]] real getMass() const;

        [[nodiscard]] Vector3 getTorqueAccum();
        [[nodiscard]] Quaternion getOrientation();
        [[nodiscard]] Vector3 getVelocity() const;
        [[nodiscard]] Matrix4 getTransform() const;
    };
}
#endif //DYNAHEX_BODY_H
