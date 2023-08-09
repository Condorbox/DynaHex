#include <iostream>
#include <random>
#include <dynaHex/particle.h>

using namespace DynaHex;

int main() {
    std::cout << "Hello, World!" << std::endl;

    Particle particleSimple, particleComplex;
    real deltaTime = (real)0.5;

    particleSimple.setPosition(1, 1, 1);
    particleSimple.setAcceleration(2, 2, 2);
    particleSimple.setInverseMass((real)0.5);
    particleSimple.setDamping((real)0.999);

    particleComplex.setPosition(1, 1, 1);
    particleComplex.setAcceleration(2, 2, 2);
    particleComplex.setInverseMass((real)0.5);
    particleComplex.setDamping((real)0.999);

    std::random_device rd;
    std::mt19937 gen(rd());

    real min_value = 0.00694;
    real max_value = 0.06667;
    std::uniform_real_distribution<real> dist(min_value, max_value);

    for (int i = 0; i < 10; i++) {
        deltaTime = dist(gen);

        particleSimple.setPosition(particleSimple.getPosition() * particleSimple.getDamping() + particleSimple.getAcceleration() * deltaTime);

        particleComplex.setPosition(particleComplex.getPosition() * real_pow(particleComplex.getDamping(), deltaTime) + particleComplex.getAcceleration() * deltaTime);

        std::cout << "deltaTime: " << deltaTime << std::endl;

        std::cout << "Position of Particle Simple: " << particleSimple.getPosition().x << ", "
                  << particleSimple.getPosition().y << ", " << particleSimple.getPosition().z << std::endl;

        std::cout << "Position of Particle Complex: " << particleComplex.getPosition().x << ", "
                  << particleComplex.getPosition().y << ", " << particleComplex.getPosition().z << std::endl;

        std::cout << "Difference: " << std::abs(particleSimple.getPosition().magnitude() - particleComplex.getPosition().magnitude()) << std::endl;

        std::cout << "__________________________________________________________________" << std::endl;
    }

    return 0;
}
