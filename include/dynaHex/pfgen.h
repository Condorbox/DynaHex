#ifndef DYNAHEX_PFGEN_H
#define DYNAHEX_PFGEN_H

#include <dynaHex/particle.h>
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
}

#endif //DYNAHEX_PFGEN_H
