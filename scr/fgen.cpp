
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
