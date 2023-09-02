
#ifndef DYNAHEX_PLINKS_H
#define DYNAHEX_PLINKS_H

#include "pcontacts.h"

namespace dynahex {
    /**
    * Links connect two particles together, generating a contact if
    * they violate the constraints of their link. It is used as a
    * base class for cables and rods, and could be used as a base
    * class for springs with a limit to their extension.
    */
    class ParticleLink : public ParticleContactGenerator {
    public:
        /**
        * Holds the pair of particles that are connected by this link.
        */
        Particle *particle[2];
    protected:
        /**
        * Returns the current length of the link.
        */
        real currentLength() const;

    public:
        /**
        * Generates the contacts to keep this link from being
        * violated. This class can only ever generate a single
        * contact, so the pointer can be a pointer to a single
        * element, the limit parameter is assumed to be at least 1
        * (0 isn’t valid), and the return value is 0 if the
                * cable wasn’t overextended, or 1 if a contact was needed.
        *
        * NB: This method is declared in the same way (as pure
        * virtual) in the parent class, but is replicated here for
        * documentation purposes.
        */
        unsigned addContact(ParticleContact *contact, unsigned limit) const override = 0;
    };

    /**
    * Cables link a pair of particles, generating a contact if they
    * stray too far apart.
    */
    class ParticleCable : public ParticleLink {
    public:
        /**
        * Holds the maximum length of the cable.
        */
        real maxLength;
        /**
        * Holds the restitution (bounciness) of the cable.
        */
        real restitution;

    public:
        /**
        * Fills the given contact structure with the contact needed
        * to keep the cable from overextending.
        */
        unsigned addContact(ParticleContact *contact, unsigned limit) const override;
    };

    /**
    * Rods link a pair of particles, generating a contact if they
    * stray too far apart or too close.
    */
    class ParticleRod : public ParticleLink {
    public:
        /**
        * Holds the length of the rod.
        */
        real length;
    public:
        /**
        * Fills the given contact structure with the contact needed
        * to keep the rod from extending or compressing.
        */
        unsigned addContact(ParticleContact *contact, unsigned limit) const override;
    };

    class MaxMinContactGenerator : public ParticleLink {
        real min;
        real max;
    public:
        unsigned addContact(ParticleContact *contact, unsigned limit) const override;
    };
}
#endif //DYNAHEX_PLINKS_H
