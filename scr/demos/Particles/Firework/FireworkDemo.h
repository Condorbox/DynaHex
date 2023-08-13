
#ifndef DYNAHEX_FIREWORKDEMO_H
#define DYNAHEX_FIREWORKDEMO_H

#include "../../Application.h"
#include "Firework.cpp"

class FireworkDemo : public Application {
private:
    /**
    * Holds the maximum number of fireworks that can be in use.
    */
    const static unsigned maxFireworks = 1024;

    /** Holds the firework data. */
    Firework fireworks[maxFireworks];

    /** Holds the index of the next firework slot to use. */
    unsigned nextFirework{};

    /** And the number of rules. */
    const static unsigned ruleCount = 9;

    /** Holds the set of rules. */
    FireworkRule rules[ruleCount];

    /** Dispatches a firework from the origin. */
    void create(unsigned type, const Firework *parent = nullptr);

    /** Dispatches the given number of fireworks from the given parent. */
    void create(unsigned type, unsigned number, const Firework *parent);

    /** Creates the rules. */
    void initFireworkRules();
public:
    FireworkDemo();
    ~FireworkDemo();
    /** Update the particle positions. */
    void update() override;

    /** Display the particle positions. */
    void display() override;
};

#endif //DYNAHEX_FIREWORKDEMO_H
