#include <iostream>
#include <random>
#include "scr/demos/Particles/Ballistic/BallisticDemo.h"

using namespace dynahex;

int main() {
    std::cout << "Hello World" << std::endl;

    unsigned long timesToDisplay = 100;

    auto* app = new BallisticDemo();

    app->changeAmmoType(3);

    for (unsigned long i = 0; i < timesToDisplay; i++) {
        app->update();
        app->display();
        std::cout << "___________________________________________" << std::endl;
    }

    return 0;
}
