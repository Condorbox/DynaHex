
#include <dynaHex/particle.h>
#include <dynaHex/random.h>

static dynahex::Random drandom;

class Firework : public dynahex::Particle {
public:
    unsigned type{};
    /**
    * The age of a firework determines when it detonates. Age gradually
    * decreases; when it passes zero the firework delivers its payload.
    * Think of age as fuse left.
    */
    dynahex::real age{};

    /**
    * Updates the firework by the given duration of time. Returns true
    * if the firework has reached the end of its life and needs to be
    * removed.
    */
    bool update(dynahex::real duration)
    {
        // Update our physical state.
        integrate(duration);
        // We work backward from our age to zero.
        age -= duration;
        return (age < 0) || (position.y < 0);
    }
};

struct FireworkRule {
    unsigned type{};
    dynahex::real minAge{};
    dynahex::real maxAge{};
    dynahex::Vector3 minVelocity;
    dynahex::Vector3 maxVelocity;
    dynahex::real damping{};

    /**
    * The payload is the new firework type to create when this
    * firework’s fuse is over.
    */
    struct Payload {
        unsigned type;
        unsigned count;
        void set(unsigned type, unsigned count)
        {
            Payload::type = type;
            Payload::count = count;
        }
    };

    unsigned payloadCount;
    /** The set of payloads. */
    Payload *payloads;
    FireworkRule() : payloadCount(0),payloads(nullptr)
    {
    }

    void init(unsigned payloadCount)
    {
        FireworkRule::payloadCount = payloadCount;
        payloads = new Payload[payloadCount];
    }

    ~FireworkRule()
    {
        if (payloads != nullptr) delete[] payloads;
    }

    /**
     * Set all the rule parameters in one go.
     */
    void setParameters(unsigned type, dynahex::real minAge, dynahex::real maxAge,
                       const dynahex::Vector3 &minVelocity, const dynahex::Vector3 &maxVelocity,
                       dynahex::real damping)
    {
        FireworkRule::type = type;
        FireworkRule::minAge = minAge;
        FireworkRule::maxAge = maxAge;
        FireworkRule::minVelocity = minVelocity;
        FireworkRule::maxVelocity = maxVelocity;
        FireworkRule::damping = damping;
    }

    /**
     * Creates a new firework of this type and writes it into the given
     * instance. The optional parent firework is used to base position
     * and velocity on.
     */
    void create(Firework *firework, const Firework *parent = NULL) const
    {
        firework->type = type;
        firework->age = drandom.randomReal(minAge, maxAge);

        dynahex::Vector3 vel;
        if (parent) {
            // The position and velocity are based on the parent.
            firework->setPosition(parent->getPosition());
            vel += parent->getVelocity();
        }
        else
        {
            dynahex::Vector3 start;
            int x = (int)drandom.randomInt(3) - 1;
            start.x = 5.0f * dynahex::real(x);
            firework->setPosition(start);
        }

        vel += drandom.randomVector(minVelocity, maxVelocity);
        firework->setVelocity(vel);

        // We use a mass of one in all cases (no point having fireworks
        // with different masses, since they are only under the influence
        // of gravity).
        firework->setMass(1);

        firework->setDamping(damping);

        firework->setAcceleration(dynahex::Vector3::GRAVITY);

        firework->clearAccumulator();
    }
};