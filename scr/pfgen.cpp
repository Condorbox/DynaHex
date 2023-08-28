
#include <dynaHex/pfgen.h>

using namespace dynahex;

void ParticleForceRegistry::updateForces(dynahex::real duration) {
    Registry::iterator i = registrations.begin();
    while (i != registrations.end()) {
        i->fg->updateForce(i->particle, duration);
        i++;
    }
}

void ParticleGravity::updateForce(Particle* particle, real duration) {
// Check that we do not have infinite mass.
    if (!particle->hasFiniteMass()) return;
// Apply the mass-scaled force to the particle.
    particle->addForce(gravity * particle->getMass());
}

void ParticleDrag::updateForce(Particle* particle, real duration) {
    Vector3 force;
    particle->getVelocity(&force);
    // Calculate the total drag coefficient.
    real dragCoeff = force.magnitude();
    dragCoeff = k1 * dragCoeff + k2 * dragCoeff * dragCoeff;
    // Calculate the final force and apply it.
    force.normalize();
    force *= -dragCoeff;
    particle->addForce(force);
}

void ParticleUplift::updateForce(Particle *particle, real duration) {
    Vector3 particlePosition;
    particle->getPosition(&particlePosition);
    if (real_abs(origin.x - particlePosition.x) > distance || real_abs(origin.z - particlePosition.z) > distance) return;

    // Calculate uplift force
    Vector3 upliftForce = Vector3(0, (gravity * particle->getMass()).magnitude(), 0);

    particle->addForce(upliftForce);
}

void ParticleAirbrake::updateForce(Particle *particle, real duration) {
    if (!isAirbrakeOn) return;

    Vector3 force;
    particle->getVelocity(&force);

    // Calculate the drag force.
    real dragForceMagnitude = -0.5 * dragCoeff * real_pow(force.magnitude(), 2);
    force.normalize();
    force *= dragForceMagnitude;

    particle->addForce(force);
}

void ParticleAttraction::updateForce(Particle *particle, real duration) {
    Vector3 force = attractionPoint - particle->getPosition();

    real distanceSquared = real_pow(force.magnitude(), 2);
    if (distanceSquared == 0) return; // Avoid division by zero.

    // Calculate the attraction force magnitude scaled by the square of the distance.
    real attractionForceMagnitude = (gravitationalConstant * particle->getMass()) / distanceSquared;

    force.normalize();
    force *= attractionForceMagnitude;

    particle->addForce(force);
}

void ParticleSpring::updateForce(Particle* particle, real duration) {
    // Calculate the vector of the spring.
    Vector3 force;
    particle->getPosition(&force);
    force -= other->getPosition();
    // Calculate the magnitude of the force.
    real magnitude = force.magnitude();
    magnitude = real_abs(magnitude - restLength);
    magnitude *= springConstant;
    // Calculate the final force and apply it.
    force.normalize();
    force *= -magnitude;
    particle->addForce(force);
}

void ParticleAnchoredSpring::updateForce(Particle *particle, real duration) {
    // Calculate the vector of the spring.
    Vector3 force;
    particle->getPosition(&force);
    force -= *anchor;
    // Calculate the magnitude of the force.
    real magnitude = force.magnitude();
    magnitude = (restLength - magnitude) * springConstant;
    // Calculate the final force and apply it.
    force.normalize();
    force *= magnitude;
    particle->addForce(force);
}

void ParticleBungee::updateForce(Particle *particle, real duration) {
    // Calculate the vector of the spring.
    Vector3 force;
    particle->getPosition(&force);
    force -= other->getPosition();
    // Check if the bungee is compressed.
    real magnitude = force.magnitude();
    if (magnitude <= restLength) return;
    // Calculate the magnitude of the force.
    magnitude = springConstant * (restLength - magnitude);
    // Calculate the final force and apply it.
    force.normalize();
    force *= -magnitude;
    particle->addForce(force);
}

void ParticleBuoyancy::updateForce(Particle *particle, real duration) {
    // Calculate the submersion depth.
    real depth = particle->getPosition().y;
    // Check if we’re out of the water.
    if (depth >= waterHeight + maxDepth) return;
    Vector3 force(0,0,0);
    // Check if we’re at maximum depth.
    if (depth <= waterHeight - maxDepth) {
        force.y = liquidDensity * volume;
        particle->addForce(force);
        return;
    }
    // Otherwise we are partly submerged.
    force.y = liquidDensity * volume * (depth - maxDepth - waterHeight) / 2 * maxDepth;
    particle->addForce(force);
}