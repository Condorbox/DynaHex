
#ifndef DYNAHEX_FGEN_H
#define DYNAHEX_FGEN_H

#include "body.h"
#include <vector>

namespace dynahex {
    class ForceGenerator {
    public:
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

    /**
    * A force generator that applies an aerodynamic force.
    */
    class Aero : public ForceGenerator {
    protected:
        /**
        * Holds the aerodynamic tensor for the surface in body
        * space.
        */
        Matrix3 tensor;
        /**
        * Holds the relative position of the aerodynamic surface in
        * body coordinates.
        */
        Vector3 position;
        /**
        * Holds a pointer to a vector containing the wind speed of the
        * environment. This is easier than managing a separate
        * wind speed vector per generator and having to update it
        * manually as the wind changes.
        */
        const Vector3* windspeed;
    public:
        /**
        * Creates a new aerodynamic force generator with the
        * given properties.
        */
        Aero(const Matrix3 &tensor, const Vector3 &position, const Vector3 *windspeed);
        /**
        * Applies the force to the given rigid body.
        */
        void updateForce(RigidBody *body, real duration) override;
    protected:
        /**
        * Uses an explicit tensor matrix to update the force on
        * the given rigid body. This is exactly the same as for updateForce,
        * except that it takes an explicit tensor.
        */
        virtual void updateForceFromTensor(RigidBody *body, real duration, const Matrix3 &tensor);
    };

    /**
    * A force generator with a control aerodynamic surface. This
    * requires three inertia tensors, for the two extremes and
    * ‘‘resting’’ position of the control surface. The latter tensor is
    * the one inherited from the base class, while the two extremes are
    * defined in this class.
    */
    class AeroControl : public Aero {
    protected:
        /**
        * The aerodynamic tensor for the surface when the control is at
        * its maximum value.
        */
        Matrix3 maxTensor;
        /**
        * The aerodynamic tensor for the surface when the control is at
        * its minimum value.
        */
        Matrix3 minTensor;
        /**
        * The current position of the control for this surface. This
        * should range between -1 (in which case the minTensor value
        * is used), through 0 (where the base-class tensor value is
        * used) to +1 (where the maxTensor value is used).
        */
        real controlSetting;
    private:
        /**
        * Calculates the final aerodynamic tensor for the current
        * control setting.
        */
        Matrix3 getTensor();
    public:
        /**
        * Creates a new aerodynamic control surface with the given
        * properties.
        */
        AeroControl(const Matrix3 &base,
                    const Matrix3 &min, const Matrix3 &max,
                    const Vector3 &position, const Vector3 *windspeed);
        /**
         * Sets the control position of this control.
         * This should range between -1 (in which case the minTensor value is used),
         * through 0 (where the base-class tensor value is used)
         * to +1 (where the maxTensor value is used).
         * Values outside that range give undefined results.
        */
        void setControl(real value);
        /**
         * Applies the force to the given rigid body.
         */
        void updateForce(RigidBody *body, real duration) override;
    };

    /**
    * A force generator to apply a buoyant force to a rigid body.
    */
    class Buoyancy : public ForceGenerator {
        /**
        * The maximum submersion depth of the object before
        * it generates its maximum buoyancy force.
        */
        real maxDepth;
        /**
        * The volume of the object.
        */
        real volume;
        /**
        * The height of the water plane above y=0. The plane will be
        * parallel to the XZ plane.
        */
        real waterHeight;
        /**
        * The density of the liquid. Pure water has a density of
        * 1000 kg per cubic meter.
        */
        real liquidDensity;
        /**
        * The center of buoyancy of the rigid body, in body coordinates.
        */
        Vector3 centreOfBuoyancy;
    public:
        /** Creates a new buoyancy force with the given parameters. */
        Buoyancy(const Vector3 &cOfB, real maxDepth, real volume, real waterHeight, real liquidDensity = 1000.0f);
        /**
        * Applies the force to the given rigid body.
        */
        void updateForce(RigidBody *body, real duration) override;
    };

    /**
    * A force generator with an aerodynamic surface that can be
    * reoriented relative to its rigid body.
    */
    class AngledAero : public Aero {
        /**
        * Holds the orientation of the aerodynamic surface relative
        * to the rigid body to which it is attached.
        */
        Quaternion orientation;
    public:
        /**
        * Creates a new aerodynamic surface with the given properties.
        */
        AngledAero(const Matrix3 &tensor, const Vector3 &position, const Vector3 *windspeed);
        /**
        * Sets the relative orientation of the aerodynamic surface
        * relative to the rigid body that it is attached to. Note that
        * this doesn’t affect the point of connection of the surface
        * to the body.
        */
        void setOrientation(const Quaternion &quat);
        /**
        * Applies the force to the given rigid body.
        */
        void updateForce(RigidBody *body, real duration) override;
    };

    class ForceRegistry {
    protected:
        /**
       * Keeps track of one force generator and the body it
       * applies to.
       */
        struct ForceRegistration
        {
            RigidBody* body;
            ForceGenerator* fg;
        };

        /**
        * Holds the list of registrations.
        */
        typedef std::vector<ForceRegistration> Registry;
        Registry registrations;
    public:
        /**
        * Registers the given force generator to apply to the
        * given body.
        */
        void add(RigidBody* body, ForceGenerator* fg);
        /**
        * Removes the given registered pair from the registry.
        * If the pair is not registered, this method will have
        * no effect.
        */
        void remove(RigidBody* body, ForceGenerator* fg);
        /**
        * Clears all registrations from the registry. This will
        * not delete the bodies or the force generators
        * themselves, just the records of their connection.
        */
        void clear();
        /**
        * Calls all the force generators to update the forces of
        * their corresponding bodies.
        */
        void updateForces(real duration);
    };
}
#endif //DYNAHEX_FGEN_H
