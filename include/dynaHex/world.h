
#ifndef DYNAHEX_WORLD_H
#define DYNAHEX_WORLD_H

#include <vector>
#include "body.h"

namespace dynahex {
    /**
    * The world represents an independent simulation of physics.
    * It keeps track of a set of rigid bodies, and provides the means to
    * update them all.
    */
    class world {
    public:
        typedef std::vector<RigidBody*> RigidBodies;
    protected:
        /**
        * Holds the rigid bodies being simulated.
        */
        RigidBodies bodies;
    public:
        /**
        * Initializes the world for a simulation frame. This clears
        * the force and torque accumulators for bodies in the
        * world. After calling this, the bodies can have their forces
        * and torques for this frame added.
        */
        void startFrame();
        /**
        * Processes all the physics for the world.
        */
        void runPhysics(real duration);
    };
}

#endif //DYNAHEX_WORLD_H
