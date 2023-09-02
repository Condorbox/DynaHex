
#include "dynaHex/pworld.h"

using namespace dynahex;

unsigned ParticleWorld::generateContacts() {
    unsigned limit = maxContacts;
    ParticleContact *nextContact = contacts;
    for (auto &contactGenerator : contactGenerators) {
        unsigned used = contactGenerator->addContact(nextContact, limit);
        limit -= used;
        nextContact += used;
        // We’ve run out of contacts to fill. This means we’re missing
        // contacts.
        if (limit <= 0) break;
    }
    // Return the number of contacts used.
    return maxContacts - limit;
}

void ParticleWorld::integrate(real duration) {
    for (auto p : particles) {
        // Integrate the particle by the given duration.
        p->integrate(duration);
    }
}

void ParticleWorld::runPhysics(real duration) {
    // First, apply the force generators.
    registry.updateForces(duration);
    // Then integrate the objects.
    integrate(duration);
    // Generate contacts.
    unsigned usedContacts = generateContacts();
    // And process them.
    if (usedContacts) {
        if (calculateIterations) resolver.setIterations(usedContacts * 2);
        resolver.resolveContacts(contacts, usedContacts, duration);
    }
}
/*
void loop()
{
    while (true) {
    // Prepare the objects for this frame.
        world.startFrame();
    // Calls to other parts of the game code.
        runGraphicsUpdate();
        updateCharacters();
    // Update the physics.
        world.runPhysics();
        if (gameOver) break;
    }
}
*/
