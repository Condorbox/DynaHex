
#include <iostream>
#include "BallisticDemo.h"

void BallisticDemo::fire() {
    // Find the first available round.
    AmmoRound *shot;
    for (shot = ammo; shot < ammo+ammoRounds; shot++)
    {
        if (shot->type == UNUSED) break;
    }

    // If we didn't find a round, then exit - we can't fire.
    if (shot >= ammo+ammoRounds) return;

    // Set the properties of the particle
    switch(currentShotType)
    {
        case PISTOL:
            shot->particle.setMass(2.0f); // 2.0kg
            shot->particle.setVelocity(0.0f, 0.0f, 35.0f); // 35m/s
            shot->particle.setAcceleration(0.0f, -1.0f, 0.0f);
            shot->particle.setDamping(0.99f);
            break;

        case ARTILLERY:
            shot->particle.setMass(200.0f); // 200.0kg
            shot->particle.setVelocity(0.0f, 30.0f, 40.0f); // 50m/s
            shot->particle.setAcceleration(0.0f, -20.0f, 0.0f);
            shot->particle.setDamping(0.99f);
            break;

        case FIREBALL:
            shot->particle.setMass(1.0f); // 1.0kg - mostly blast damage
            shot->particle.setVelocity(0.0f, 0.0f, 10.0f); // 5m/s
            shot->particle.setAcceleration(0.0f, 0.6f, 0.0f); // Floats up
            shot->particle.setDamping(0.9f);
            break;

        case LASER:
            // Note that this is the kind of laser bolt seen in films,
            // not a realistic laser beam!
            shot->particle.setMass(0.1f); // 0.1kg - almost no weight
            shot->particle.setVelocity(0.0f, 0.0f, 100.0f); // 100m/s
            shot->particle.setAcceleration(0.0f, 0.0f, 0.0f); // No gravity
            shot->particle.setDamping(0.99f);
            break;
        default:
            break;
    }

    // Set the data common to all particle types
    shot->particle.setPosition(0.0f, 1.5f, 0.0f);
    shot->startTime = 0.0f;
    shot->type = currentShotType;

    // Clear the force accumulators
    shot->particle.clearAccumulator();
}

void BallisticDemo::update() {
    fire();

    // Find the duration of the last frame in seconds
    float duration = 3.45f;
    if (duration <= 0.0f) return;

    // Update the physics of each particle in turn
    for (AmmoRound *shot = ammo; shot < ammo+ammoRounds; shot++)
    {
        if (shot->type != UNUSED)
        {
            // Run the physics
            shot->particle.integrate(duration);

            // Check if the particle is now invalid
            if (shot->particle.getPosition().y < 0.0f ||
                shot->particle.getPosition().z > 200.0f)
            {
                // We simply set the shot type to be unused, so the
                // memory it occupies can be reused by another shot.
                shot->type = UNUSED;
            }
        }
    }

}

void BallisticDemo::display() {
    for (AmmoRound *shot = ammo; shot < ammo+ammoRounds; shot++)
    {
        if (shot->type != UNUSED)
        {
            shot->render();
        }
    }
}

void BallisticDemo::changeAmmoType(unsigned short int number) {
    switch(number)
    {
        case 1: currentShotType = PISTOL; break;
        case 2: currentShotType = ARTILLERY; break;
        case 3: currentShotType = FIREBALL; break;
        case 4: currentShotType = LASER; break;
        default:
            break;
    }
}

BallisticDemo::BallisticDemo() {
    currentShotType = PISTOL;

    for (auto & i : ammo) {
        i.type = currentShotType;
    }
}
