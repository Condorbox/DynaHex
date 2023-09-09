#include "dynaHex/world.h"
#include "dynaHex/pworld.h"

using namespace dynahex;

void world::startFrame() {
    for (auto b : bodies) {
        b->clearAccumulators();
        b->calculateDerivedData();
    }
}

void world::runPhysics(real duration) {
    for(auto b : bodies) {
        b->integrate(duration);
    }
}
