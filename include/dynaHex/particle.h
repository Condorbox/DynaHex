#ifndef DYNAHEX_PARTICLE_H
#define DYNAHEX_PARTICLE_H

#include "core.h"

namespace DynaHex {
    class Particle {
    protected:
        /**
         * Holds the inverse of the mass of the particle. It
         * is more useful to hold the inverse mass because
         * integration is simpler, and because in real time
         * simulation it is more useful to have objects with
         * infinite mass (immovable) than zero mass
         * (completely unstable in numerical simulation).
         */
        real inverseMass;
        real damping; //0 remove all energy, 1 no remove energy
        Vector3 position;
        Vector3 velocity;
        Vector3 acceleration;

    public:
        void setInverseMass(real inverseMass);
        [[nodiscard]] real getInverseMass() const;
        real getMass() const;
        /**
        * Returns true if the mass of the particle is not-infinite.
        */
        bool hasFiniteMass() const;
        void setDamping(const real damping);
        real getDamping() const;
        void setPosition(const Vector3 &position);
        void setPosition(const real x, const real y, const real z);
        void getPosition(Vector3 *position) const;
        Vector3 getPosition() const;
        void setVelocity(const Vector3 &velocity);
        void setVelocity(const real x, const real y, const real z);
        Vector3 getVelocity() const;
        void setAcceleration(const Vector3 &acceleration);
        void setAcceleration(const real x, const real y, const real z);
        Vector3 getAcceleration() const;

        /**
        * Integrates the particle forward in time by the given amount.
        * This function uses a Newton-Euler integration method, which is a
        * linear approximation to the correct integral. For this reason it
        * may be inaccurate in some cases.
        */
        void integrate(real duration);
        real calculateKineticEnergy();
        void clearAccumulator();
    };
}

#endif //DYNAHEX_PARTICLE_H
