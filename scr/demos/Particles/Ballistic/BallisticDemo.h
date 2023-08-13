
#ifndef DYNAHEX_BALLISTICDEMO_H
#define DYNAHEX_BALLISTICDEMO_H

#include "dynaHex/particle.h"
#include "../../Application.h"

class BallisticDemo : public Application {

    enum ShotType {
        UNUSED = 0,
        PISTOL,
        ARTILLERY,
        FIREBALL,
        LASER
    };

    struct AmmoRound {
        dynahex::Particle particle;
        ShotType type;
        unsigned startTime;

        void render() {
            dynahex::Vector3 position;

            particle.getPosition(&position);
            std::cout << "type -> " << type << " position: " << position.x << ", "
                      << position.y << ", " << position.z << std::endl;
        }
    };

    const static unsigned ammoRounds = 16;
    AmmoRound ammo[ammoRounds];
    ShotType currentShotType;

public:
    BallisticDemo();

    /** Update the particle positions. */
    void update() override;

    /** Display the particle positions. */
    void display() override;

    void changeAmmoType(unsigned short int number);
    void fire();
};

#endif //DYNAHEX_BALLISTICDEMO_H
