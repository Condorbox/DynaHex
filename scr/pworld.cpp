
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

ParticleWorld::ParticleWorld(unsigned int maxContacts, unsigned int iterations) : resolver(iterations), maxContacts(maxContacts) {
    contacts = new ParticleContact[maxContacts];
    calculateIterations = (iterations == 0);
}

ParticleWorld::~ParticleWorld() {
    delete[] contacts;
}

void ParticleWorld::startFrame() {
    for (auto p : particles) {
        p->clearAccumulator();
    }
}

ParticleWorld::Particles &ParticleWorld::getParticles() {
    return particles;
}

ParticleWorld::ContactGenerators& ParticleWorld::getContactGenerators() {
    return contactGenerators;
}

ParticleForceRegistry& ParticleWorld::getForceRegistry() {
    return registry;
}

/* How the main loop might look
void loop() {
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

void GroundContacts::init(dynahex::ParticleWorld::Particles *particles) {
    GroundContacts::particles = particles;
}

unsigned GroundContacts::addContact(ParticleContact *contact, unsigned int limit) const {
    unsigned count = 0;
    for (auto p : *particles) {
        dynahex::real y = p->getPosition().y;
        if (y < (real)0.0) {
            contact->contactNormal = dynahex::Vector3::UP;
            contact->particle[0] = p;
            contact->particle[1] = nullptr;
            contact->penetration = -y;
            contact->restitution = (real)0.2;
            contact++;
            count++;
        }

        if (count >= limit) return count;
    }

    return count;
}
