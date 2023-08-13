
#include "FireworkDemo.h"

FireworkDemo::FireworkDemo() = default;
FireworkDemo::~FireworkDemo() = default;

void FireworkDemo::create(unsigned int type, const Firework *parent) {

}

void FireworkDemo::create(unsigned int type, unsigned int number, const Firework *parent) {

}

void FireworkDemo::initFireworkRules() {
    // Go through the firework types and create their rules.
    rules[0].init(2);
    rules[0].setParameters(
            1, // type
            0.5f, 1.4f, // age range
            dynahex::Vector3(-5, 25, -5), // min velocity
            dynahex::Vector3(5, 28, 5), // max velocity
            0.1 // damping
    );
    rules[0].payloads[0].set(3, 5);
    rules[0].payloads[1].set(5, 5);

    rules[1].init(1);
    rules[1].setParameters(
            2, // type
            0.5f, 1.0f, // age range
            dynahex::Vector3(-5, 10, -5), // min velocity
            dynahex::Vector3(5, 20, 5), // max velocity
            0.8 // damping
    );
    rules[1].payloads[0].set(4, 2);

    rules[2].init(0);
    rules[2].setParameters(
            3, // type
            0.5f, 1.5f, // age range
            dynahex::Vector3(-5, -5, -5), // min velocity
            dynahex::Vector3(5, 5, 5), // max velocity
            0.1 // damping
    );

    rules[3].init(0);
    rules[3].setParameters(
            4, // type
            0.25f, 0.5f, // age range
            dynahex::Vector3(-20, 5, -5), // min velocity
            dynahex::Vector3(20, 5, 5), // max velocity
            0.2 // damping
    );

    rules[4].init(1);
    rules[4].setParameters(
            5, // type
            0.5f, 1.0f, // age range
            dynahex::Vector3(-20, 2, -5), // min velocity
            dynahex::Vector3(20, 18, 5), // max velocity
            0.01 // damping
    );
    rules[4].payloads[0].set(3, 5);

    rules[5].init(0);
    rules[5].setParameters(
            6, // type
            3, 5, // age range
            dynahex::Vector3(-5, 5, -5), // min velocity
            dynahex::Vector3(5, 10, 5), // max velocity
            0.95 // damping
    );

    rules[6].init(1);
    rules[6].setParameters(
            7, // type
            4, 5, // age range
            dynahex::Vector3(-5, 50, -5), // min velocity
            dynahex::Vector3(5, 60, 5), // max velocity
            0.01 // damping
    );
    rules[6].payloads[0].set(8, 10);

    rules[7].init(0);
    rules[7].setParameters(
            8, // type
            0.25f, 0.5f, // age range
            dynahex::Vector3(-1, -1, -1), // min velocity
            dynahex::Vector3(1, 1, 1), // max velocity
            0.01 // damping
    );

    rules[8].init(0);
    rules[8].setParameters(
            9, // type
            3, 5, // age range
            dynahex::Vector3(-15, 10, -5), // min velocity
            dynahex::Vector3(15, 15, 5), // max velocity
            0.95 // damping
    );
    // ... and so on for other firework types ...
}

void FireworkDemo::update() {
    // Find the duration of the last frame in seconds
    float duration = 0.00694f; // 144 FPS
    if (duration <= 0.0f) return;

    for (Firework *firework = fireworks;
         firework < fireworks+maxFireworks;
         firework++)
    {
        // Check if we need to process this firework.
        if (firework->type > 0)
        {
            // Does it need removing?
            if (firework->update(duration))
            {
                // Find the appropriate rule
                FireworkRule *rule = rules + (firework->type-1);

                // Delete the current firework (this doesn't affect its
                // position and velocity for passing to the create function,
                // just whether or not it is processed for rendering or
                // physics.
                firework->type = 0;

                // Add the payload
                for (unsigned i = 0; i < rule->payloadCount; i++)
                {
                    FireworkRule::Payload * payload = rule->payloads + i;
                    create(payload->type, payload->count, firework);
                }
            }
        }
    }
}

void FireworkDemo::display() {

}
