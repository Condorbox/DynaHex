#ifndef DYNAHEX_PCONTACTS_H
#define DYNAHEX_PCONTACTS_H

#include "particle.h"

namespace dynahex {
    /**
    * A contact represents two objects in contact (in this case
    * ParticleContact representing two particles). Resolving a
    * contact removes their interpenetration, and applies sufficient
    * impulse to keep them apart. Colliding bodies may also rebound.
    *
    * The contact has no callable functions, it just holds the
    * contact details. To resolve a set of contacts, use the particle
    * contact resolver class.
    */

    class ParticleContact {
    public:
        /**
        * Holds the particles that are involved in the contact. The
        * second of these can be NULL for contacts with the scenery.
        */
        Particle *particle[2];
        /**
        * Holds the normal restitution coefficient at the contact.
        */
        real restitution;
        /**
        * Holds the direction of the contact in world coordinates.
        * (from the first object's perspective)
        */
        Vector3 contactNormal;
        /**
        * Holds the depth of penetration at the contact.
        */
        real penetration;
        /**
         * Holds the amount each particle is moved by during interpenetration resolution.
         */
        Vector3 particleMovement[2];
    public:
        /**
        * Resolves this contact for both velocity and interpenetration.
        */
        void resolve(real duration);

        /**
        * Calculates the separating velocity at this contact.
        */
        [[nodiscard]] real calculateSeparatingVelocity() const;

    private:
        /**
        * Handles the impulse calculations for this collision.
        */
        void resolveVelocity(real duration);

        /**
         * Handles the interpenetration resolution for this contact.
         */
        void resolveInterpenetration(real duration);
    };

    /**
    * The contact resolution routine for particle contacts. One
    * resolver instance can be shared for the entire simulation.
    */
    class ParticleContactResolver {
    protected:
        /**
        * Holds the number of iterations allowed.
        */
        unsigned iterations;
        /**
        * This is a performance tracking value; we keep a record
        * of the actual number of iterations used.
        */
        unsigned iterationsUsed;
    public:
        /**
        * Creates a new contact resolver.
        */
        explicit ParticleContactResolver(unsigned iterations);
        /**
        * Sets the number of iterations that can be used.
        */
        void setIterations(unsigned iterations);
        /**
        * Resolves a set of particle contacts for both penetration
        * and velocity.
        */
        void resolveContacts(ParticleContact *contactArray, unsigned numContacts, real duration);
    };

    /**
     * This is the basic polymorphic interface for contact generators
     * applying to particles.
     */
    class ParticleContactGenerator {
    public:
        /**
         * Fills the given contact structure with the generated
         * contact. The contact pointer should point to the first
         * available contact in a contact array, where limit is the
         * maximum number of contacts in the array that can be written
         * to. The method returns the number of contacts that have
         * been written.
         */
        virtual unsigned addContact(ParticleContact *contact, unsigned limit) const = 0;
    };

    class GroundPlaneCollisionDetector : public ParticleContactGenerator {
        Vector3 ground = Vector3::ZERO;
    public:
        unsigned addContact(ParticleContact *contact, unsigned limit) const override;
    };
}
#endif //DYNAHEX_PCONTACTS_H
