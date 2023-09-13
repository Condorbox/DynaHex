
#include <algorithm>
#include "dynaHex/fgen.h"

using namespace dynahex;

Gravity::Gravity(const Vector3 &gravity) : gravity(gravity) {}

void Gravity::updateForce(RigidBody *body, real duration) {
    // Check that we do not have infinite mass.
    if (!body->hasFiniteMass()) return;
    // Apply the mass-scaled force to the body.
    body->addForce(gravity * body->getMass());
}

Spring::Spring(const Vector3 &localConnectionPt, RigidBody *other, const Vector3 &otherConnectionPt,
          real springConstant, real restLength) : connectionPoint(localConnectionPt),
          otherConnectionPoint(otherConnectionPt), other(other), springConstant(springConstant),
          restLength(restLength) {}

void Spring::updateForce(RigidBody* body, real duration) {
    // Calculate the two ends in world space
    Vector3 lws = body->getPointInWorldSpace(connectionPoint);
    Vector3 ows = other->getPointInWorldSpace(otherConnectionPoint);

    // Calculate the vector of the spring
    Vector3 force = lws - ows;

    // Calculate the magnitude of the force
    real magnitude = force.magnitude();
    magnitude = real_abs(magnitude - restLength);
    magnitude *= springConstant;

    // Calculate the final force and apply it
    force.normalize();
    force *= -magnitude;
    body->addForceAtPoint(force, lws);
}

AngularVelocityController::AngularVelocityController(const Vector3 &targetAngularVelocity, real maxDistance) :
        targetAngularVelocity(targetAngularVelocity), maxDistance(maxDistance) {}

void AngularVelocityController::updateForce(RigidBody *body, real duration) {
    // Calculate angular velocity
    Vector3 currentAngularVelocity = body->getTorqueAccum();

    // Calculate distance to target
    const real targetDistance = real_abs(currentAngularVelocity.scalarProduct(targetAngularVelocity));

    // Ensure distance is not too far
    if (targetDistance >= maxDistance) return;

    // Calculate the difference between current and target angular velocities.
    Vector3 angularVelocityDifference = targetAngularVelocity - currentAngularVelocity;

    // Calculate the torque direction.
    Vector3 torqueDirection = angularVelocityDifference;

    // Calculate the magnitude of the torque.
    real torqueMagnitude = torqueDirection.magnitude();

    torqueDirection.normalize();
    // Apply the torque to the rigid body.
    body->addForce(torqueDirection * torqueMagnitude);
}

Aero::Aero(const Matrix3 &tensor, const Vector3 &position, const Vector3 *windspeed) : tensor(tensor),
    position(position), windspeed(windspeed) {}

void Aero::updateForce(RigidBody *body, real duration) {
    Aero::updateForceFromTensor(body, duration, tensor);
}

void Aero::updateForceFromTensor(RigidBody *body, real duration, const Matrix3 &tensor) {
    //Calculate total velocity (wind speed and body’s velocity).
    Vector3 velocity = body->getVelocity();
    velocity += *windspeed;
    // Calculate the velocity in body coordinates.
    Vector3 bodyVel = body->getTransform().transformInverseDirection(velocity);
    // Calculate the force in body coordinates.
    Vector3 bodyForce = tensor.transform(bodyVel);
    Vector3 force = body->getTransform().transformDirection(bodyForce);
    // Apply the force.
    body->addForceAtBodyPoint(force, position);
}


Matrix3 AeroControl::getTensor() {
    if (controlSetting <= -1.0f) return minTensor;
    else if (controlSetting >= 1.0f) return maxTensor;
    else if (controlSetting < 0) {
        return Matrix3::linearInterpolate(minTensor, tensor, controlSetting+1.0f);
    }
    else if (controlSetting > 0) {
        return Matrix3::linearInterpolate(tensor, maxTensor, controlSetting);
    }
    else return tensor;
}

void AeroControl::updateForce(RigidBody *body, real duration) {
    Matrix3 tensor = getTensor();
    Aero::updateForceFromTensor(body, duration, tensor);
}

AeroControl::AeroControl(const Matrix3 &base, const Matrix3 &min, const Matrix3 &max, const Vector3 &position,
                         const Vector3 *windspeed) : Aero(base, position, windspeed),
                         minTensor(min), maxTensor(max), controlSetting(0.0f) {}

void AeroControl::setControl(real value) {
    AeroControl::controlSetting = value;
}

Buoyancy::Buoyancy(const Vector3 &cOfB, real maxDepth, real volume, real waterHeight, real liquidDensity) :
centreOfBuoyancy(cOfB), maxDepth(maxDepth), volume(volume), waterHeight(waterHeight), liquidDensity(liquidDensity) {}

void Buoyancy::updateForce(RigidBody *body, real duration) {
// Calculate the submersion depth
    Vector3 pointInWorld = body->getPointInWorldSpace(centreOfBuoyancy);
    real depth = pointInWorld.y;

    // Check if we're out of the water
    if (depth >= waterHeight + maxDepth) return;
    Vector3 force = Vector3::ZERO;

    // Check if we're at maximum depth
    if (depth <= waterHeight - maxDepth) {
        force.y = liquidDensity * volume;
        body->addForceAtBodyPoint(force, centreOfBuoyancy);
        return;
    }

    // Otherwise we are partly submerged
    force.y = liquidDensity * volume * (depth - maxDepth - waterHeight) / 2 * maxDepth;
    body->addForceAtBodyPoint(force, centreOfBuoyancy);
}

AngledAero::AngledAero(const Matrix3 &tensor, const Vector3 &position, const Vector3 *windspeed) :
Aero(tensor, position, windspeed) {}

void AngledAero::updateForce(RigidBody *body, real duration) {
    Aero::updateForce(body, duration);
}

void AngledAero::setOrientation(const Quaternion &quat) {
    orientation = quat;
}

void ForceRegistry::updateForces(real duration) {
    auto i = registrations.begin();
    for (; i != registrations.end(); i++) {
        i->fg->updateForce(i->body, duration);
    }
}

void ForceRegistry::add(RigidBody* body, ForceGenerator* fg) {
    ForceRegistry::ForceRegistration registration{};
    registration.body = body;
    registration.fg = fg;
    registrations.push_back(registration);
}

void ForceRegistry::clear() {
    registrations.clear();
}

void ForceRegistry::remove(RigidBody* body, ForceGenerator* fg) {
    registrations.erase(std::remove_if(registrations.begin(), registrations.end(),
        [body, fg](const ForceRegistration& registration) {
            return registration.body == body && registration.fg == fg;
        }), registrations.end());
}
