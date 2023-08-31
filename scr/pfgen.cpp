
#include <dynaHex/pfgen.h>
#include <algorithm>

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

void ParticleFakeSpring::updateForce(Particle* particle, real duration) {
    // Check that we do not have infinite mass.
    if (!particle->hasFiniteMass()) return;
    // Calculate the relative position of the particle to the anchor.
    Vector3 position;
    particle->getPosition(&position);
    position -= *anchor;
    // Calculate the constants and check that they are in bounds.
    real gamma = 0.5f * real_sqrt(4 * springConstant - damping*damping);
    if (gamma == 0.0f) return;
    Vector3 c = position * (damping / (2.0f * gamma)) + particle->getVelocity() * (1.0f / gamma);
    // Calculate the target position.
    Vector3 target = position * real_cos(gamma * duration) + c * real_sin(gamma * duration);
    target *= real_exp(-0.5f * duration * damping);
    // Calculate the resulting acceleration, and therefore the force.
    Vector3 accel = (target - position) * (1.0f / duration*duration) - particle->getVelocity() * duration;
    particle->addForce(accel * particle->getMass());
}

void ParticleSpringLimit::updateForce(Particle *particle, real duration) {
    // Check that we do not have infinite mass.
    if (!particle->hasFiniteMass()) return;
    // Calculate the relative position of the particle to the anchor.
    Vector3 position;
    particle->getPosition(&position);
    position -= *anchor;
    // Calculate if the spring is extended beyond it's limit
    if (position.magnitude() > maxDistance) {
        springConstant *= (real).10;
    }
    // Calculate the constants and check that they are in bounds.
    real gamma = 0.5f * real_sqrt(4 * springConstant - damping*damping);
    if (gamma == 0.0f) return;
    Vector3 c = position * (damping / (2.0f * gamma)) + particle->getVelocity() * (1.0f / gamma);
    // Calculate the target position.
    Vector3 target = position * real_cos(gamma * duration) + c * real_sin(gamma * duration);
    target *= real_exp(-0.5f * duration * damping);
    // Calculate the resulting acceleration, and therefore the force.
    Vector3 accel = (target - position) * (1.0f / duration*duration) - particle->getVelocity() * duration;
    particle->addForce(accel * particle->getMass());
}

void ParticleAirBuoyancy::updateForce(Particle *particle, real duration) {
    real altitude = particle->getPosition().y;

    if (altitude >= maxAltitude) {
        particle->clearAccumulator(); // No force beyond maxAltitude
        return;
    }

    real densityDifference = airDensity - objectDensity;
    real forceMagnitude = densityDifference * particle->getMass(); // F = m * (ρ_air - ρ_object)

    // Apply a maximum force to avoid excessive force near maxAltitude
    if (forceMagnitude > maxForce) {
        forceMagnitude = maxForce;
    }

    Vector3 force(0, forceMagnitude, 0);
    particle->addForce(force);
}

void ParticleOvercrowding::addParticle(Particle *particle) {
    particles.push_back(particle);
}

void ParticleOvercrowding::removeParticle(Particle *particle) {
    particles.erase(std::remove(particles.begin(), particles.end(), particle), particles.end());
}

void ParticleOvercrowding::updateForce(Particle* particle, real duration) {
    for (Particle* otherParticle : particles) {
        if (otherParticle == particle) {
            continue; // Skip self-comparison
        }

        Vector3 separation = particle->getPosition() - otherParticle->getPosition();
        real distance = separation.magnitude();

        if (distance < maxDistance) {
            real forceMagnitude = (maxDistance - distance) * springConstant;
            // Apply force in the direction of separation
            Vector3 force = (separation / distance) * forceMagnitude;
            particle->addForce(force);
        }
    }
}

void ParticleHomingBullet::updateForce(Particle *particle, real duration) {
    if (!target) {
        return; // No target to home in on
    }
    // Calculate the integrated positions of the particle and the target
    particle->integrate(duration);
    target->integrate(duration);

    Vector3 separation = particle->getPosition() - target->getPosition();
    real distance = separation.magnitude();

    // Calculate force magnitude using spring equation
    real forceMagnitude = (distance * springConstant) * duration;

    // Apply force in the direction of separation
    Vector3 force = (separation / distance) * forceMagnitude;
    particle->addForce(force);
}
