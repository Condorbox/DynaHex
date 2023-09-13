#ifndef DYNAHEX_PFGEN_H
#define DYNAHEX_PFGEN_H

#include "dynaHex/particle.h"
#include <vector>

namespace dynahex {
    class ParticleForceGenerator {
    public:
        /**
        * Overload this in implementations of the interface to calculate
        * and update the force applied to the given particle.
        */
        virtual void updateForce(Particle *particle, real duration) = 0;
    };

    /**
    * Holds all the force generators and the particles that they apply to.
    */
    class ParticleForceRegistry {
    protected:
        /**
         * Keeps track of one force generator and the particle it
        * applies to.
        */
        struct ParticleForceRegistration
        {
            Particle *particle;
            ParticleForceGenerator *fg;
        };

        /**
        * Holds the list of registrations.
        */
        typedef std::vector<ParticleForceRegistration> Registry;
        Registry registrations;

    public:
        /**
        * Registers the given force generator to apply to the
        * given particle.
        */
        void add(Particle* particle, ParticleForceGenerator *fg);
        /**
        * Removes the given registered pair from the registry.
        * If the pair is not registered, this method will have
        * no effect.
        */
        void remove(Particle* particle, ParticleForceGenerator *fg);
        /**
        * Clears all registrations from the registry. This will
        * not delete the particles or the force generators
        * themselves, just the records of their connection.
        */
        void clear();
        /**
        * Calls all the force generators to update the forces of
        * their corresponding particles.
        */
        void updateForces(real duration);
    };

    /**
    * A force generator that applies a gravitational force. One instance
    * can be used for multiple particles.
    */
    class ParticleGravity : public ParticleForceGenerator {
        Vector3 gravity;

    public:
        ParticleGravity(const Vector3 &gravity);
        void updateForce(Particle *particle, real duration) override;
    };

    /**
    * A force generator that applies a drag force. One instance
    * can be used for multiple particles.
    */
    class ParticleDrag : public ParticleForceGenerator {
        /** Holds the velocity drag coefficient. */
        real k1;
        /** Holds the velocity squared drag coefficient. */
        real k2;
    public:
        /** Creates the generator with the given coefficients. */
        ParticleDrag(real k1, real k2);
        /** Applies the drag force to the given particle. */
        void updateForce(Particle *particle, real duration) override;
    };

    class ParticleUplift : public  ParticleForceGenerator {
        Vector3 gravity;
        /** Holds the origin of the force. */
        Vector3 origin;
        /** Holds the maximus distance that the force applys*/
        real distance;

    public:
        ParticleUplift(Vector3 &gravity, Vector3 &origin, real distance);
        void updateForce(Particle *particle, real duration) override;
    };

    class ParticleAirbrake : public ParticleForceGenerator {
        bool isAirbrakeOn;
        real dragCoeff;

    public:
        ParticleAirbrake(bool isAirbrakeOn, real dragCoeff);
        void updateForce(Particle *particle, real duration) override;
    };

    class ParticleAttraction : public ParticleForceGenerator {
        Vector3 attractionPoint;
        real gravitationalConstant;

    public:
        ParticleAttraction(Vector3 attractionPoint, real gravitationalConstant);
        void updateForce(Particle *particle, real duration) override;
    };

    class ParticleSpring : public ParticleForceGenerator {
        /** The particle at the other end of the spring. */
        Particle *other;
        /** Holds the spring constant. */
        real springConstant;
        /** Holds the rest length of the spring. */
        real restLength;

    public:
        /** Creates a new spring with the given parameters. */
        ParticleSpring(Particle *other, real springConstant, real restLength);
        /** Applies the spring force to the given particle. */
        void updateForce(Particle *particle, real duration) override;
    };
    /**
    * A force generator that applies a spring force, where
    * one end is attached to a fixed point in space.
    */
    class ParticleAnchoredSpring : public ParticleForceGenerator {
    protected:
        /** The location of the anchored end of the spring. */
        Vector3 *anchor;
        /** Holds the spring constant. */
        real springConstant;
        /** Holds the rest length of the spring. */
        real restLength;
    public:
        /** Creates a new spring with the given parameters. */
        ParticleAnchoredSpring(Vector3 *anchor, real springConstant, real restLength);
        /** Applies the spring force to the given particle. */
        void updateForce(Particle *particle, real duration) override;
    };

    /**
    * A force generator that applies a spring force only
    * when extended.
    */
    class ParticleBungee : public ParticleForceGenerator {
        /** The particle at the other end of the spring. */
        Particle *other;
        /** Holds the spring constant. */
        real springConstant;
        /** Holds the length of the bungee at the point it begins to generate a force. */
        real restLength;
    public:
        /** Creates a new bungee with the given parameters. */
        ParticleBungee(Particle *other, real springConstant, real restLength);
        /** Applies the spring force to the given particle. */
        void updateForce(Particle *particle, real duration) override;
    };

    /**
    * A force generator that applies a buoyancy force for a plane of
    * liquid parallel to XZ plane.
    */
    class ParticleBuoyancy : public ParticleForceGenerator {
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
        * The height of the water plane above y = 0. The plane will be
        * parallel to the XZ plane.
        */
        real waterHeight;
        /**
        * The density of the liquid. Pure water has a density of
        * 1000 kg per cubic meter.
        */
        real liquidDensity;
    public:
        /** Creates a new buoyancy force with the given parameters. */
        ParticleBuoyancy(real maxDepth, real volume, real waterHeight, real liquidDensity = 1000.0f);
        /** Applies the buoyancy force to the given particle. */
        void updateForce(Particle *particle, real duration) override;
    };

    /**
    * A force generator that fakes a stiff spring force, and where
    * one end is attached to a fixed point in space.
    */
    class ParticleFakeSpring : public ParticleForceGenerator {
        /** The location of the anchored end of the spring. */
        Vector3 *anchor;
        /** Holds the spring constant. */
        real springConstant;
        /** Holds the damping on the oscillation of the spring. */
        real damping;
    public:
        /** Creates a new spring with the given parameters. */
        ParticleFakeSpring(Vector3 *anchor, real springConstant, real damping);
        /** Applies the spring force to the given particle. */
        void updateForce(Particle *particle, real duration) override;
    };

    class ParticleSpringLimit : public ParticleForceGenerator {
        /** The location of the anchored end of the spring. */
        Vector3 *anchor;
        /** Holds the spring constant. */
        real springConstant;
        /** Holds the damping on the oscillation of the spring. */
        real damping;
        /** Holds the limit of elasticity*/
        real maxDistance;
    public:
        ParticleSpringLimit(Vector3 *anchor, real springConstant, real damping, real maxDistance);
        void updateForce(Particle *particle, real duration) override;
    };

    class ParticleAirBuoyancy : public ParticleForceGenerator{
        real maxAltitude;   // Maximum altitude where force diminishes
        real objectDensity; // Density of the object
        real airDensity;    // Density of the surrounding air
        real maxForce;      // Maximum force at maxAltitude
    public:
        ParticleAirBuoyancy(real maxAltitude, real objectDensity, real airDensity, real maxForce);
        void updateForce(Particle *particle, real duration) override;
    };

    class ParticleOvercrowding : public ParticleForceGenerator {
        real maxDistance;          // Maximum separation distance
        real springConstant;       // Spring constant for the forces
        std::vector<Particle*> particles; // List of particles to track
    public:
        ParticleOvercrowding(real maxDistance, real springConstant);
        void addParticle(Particle* particle);
        void removeParticle(Particle* particle);
        void updateForce(Particle* particle, real duration) override;
    };

    class ParticleHomingBullet : public ParticleForceGenerator {
        Particle* target;         // The target particle to home in on
        real springConstant;      // Spring constant for the forces
    public:
        ParticleHomingBullet(Particle* target, real springConstant);
        void updateForce(Particle* particle, real duration) override;
    };
}
#endif //DYNAHEX_PFGEN_H
