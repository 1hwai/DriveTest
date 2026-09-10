#include "Solver.h"
#include "RigidBody.h"
#include "../Core/Debug/Logger.h"

#include <algorithm>
#include <cmath>

Solver::Solver()
    : m_penetrationSlop(0.001f),
    m_correctionPercent(0.2f),
    m_velocityIterations(16),
    m_restitutionThreshold(1.0f) {}

void Solver::SetPenetrationSlop(float slop) {
    m_penetrationSlop = std::max(0.0f, slop);
}

float Solver::GetPenetrationSlop() const {
    return m_penetrationSlop;
}

void Solver::SetCorrectionPercent(float percent) {
    m_correctionPercent = percent;

    if (m_correctionPercent < 0.0f)
        m_correctionPercent = 0.0f;
    else if (m_correctionPercent > 1.0f)
        m_correctionPercent = 1.0f;
}

float Solver::GetCorrectionPercent() const {
    return m_correctionPercent;
}

void Solver::SetVelocityIterations(int iterations) {
    m_velocityIterations = std::max(1, iterations);
}

int Solver::GetVelocityIterations() const {
    return m_velocityIterations;
}

void Solver::SetRestitutionThreshold(float threshold) {
    m_restitutionThreshold = std::max(0.0f, threshold);
}

float Solver::GetRestitutionThreshold() const {
    return m_restitutionThreshold;
}

void Solver::WarmStart(Contact& contact) {
    RigidBody* bodyA = contact.GetBodyA();
    RigidBody* bodyB = contact.GetBodyB();

    if (!bodyA || !bodyB)
        return;

    Vec3 normal = contact.GetNormal();
    float inverseMassA = bodyA->GetInverseMass();
    float inverseMassB = bodyB->GetInverseMass();
    Mat3 inverseInertiaA = bodyA->GetWorldInverseInertiaTensor();
    Mat3 inverseInertiaB = bodyB->GetWorldInverseInertiaTensor();

    for (int i = 0; i < contact.GetPointCount(); ++i) {
        ContactPoint& point = contact.GetPoint(i);

        if (point.normalImpulse <= 0.0f)
            continue;

        Vec3 rA = point.position - bodyA->GetPosition();
        Vec3 rB = point.position - bodyB->GetPosition();
        Vec3 impulse = normal * point.normalImpulse;

        bodyA->SetLinearVelocity(
            bodyA->GetLinearVelocity() - impulse * inverseMassA, false
        );

        bodyB->SetLinearVelocity(
            bodyB->GetLinearVelocity() + impulse * inverseMassB, false
        );

        bodyA->SetAngularVelocity(
            bodyA->GetAngularVelocity() - inverseInertiaA * rA.Cross(impulse), false
        );

        bodyB->SetAngularVelocity(
            bodyB->GetAngularVelocity() + inverseInertiaB * rB.Cross(impulse), false
        );
    }
}

void Solver::SolvePosition(Contact& contact) {
    RigidBody* bodyA = contact.GetBodyA();
    RigidBody* bodyB = contact.GetBodyB();

    if (!bodyA || !bodyB)
        return;

    float inverseMassA = bodyA->GetInverseMass();
    float inverseMassB = bodyB->GetInverseMass();
    float totalInverseMass = inverseMassA + inverseMassB;

    if (totalInverseMass <= 0.0f)
        return;

    float maxPenetration = 0.0f;

    for (int i = 0; i < contact.GetPointCount(); ++i) {
        maxPenetration = std::max(
            maxPenetration,
            contact.GetPoint(i).penetration
        );
    }

    float correctionDepth = maxPenetration - m_penetrationSlop;

    if (correctionDepth <= 0.0f)
        return;

    float correctionPercent = m_correctionPercent;

    if (correctionDepth > 0.5f)
        correctionPercent = 0.8f;
    else if (correctionDepth > 0.2f)
        correctionPercent = 0.5f;
    else if (correctionDepth > 0.05f)
        correctionPercent = 0.3f;

    if (correctionPercent < 0.0f)
        correctionPercent = 0.0f;
    else if (correctionPercent > 1.0f)
        correctionPercent = 1.0f;

    float correctionMagnitude =
        correctionDepth * correctionPercent / totalInverseMass;

    Vec3 correction = contact.GetNormal() * correctionMagnitude;

    bodyA->SetPosition(
        bodyA->GetPosition() - correction * inverseMassA
    );

    bodyB->SetPosition(
        bodyB->GetPosition() + correction * inverseMassB
    );
}

void Solver::SolveVelocity(Contact& contact) {
    RigidBody* bodyA = contact.GetBodyA();
    RigidBody* bodyB = contact.GetBodyB();

    if (!bodyA || !bodyB)
        return;

    Vec3 normal = contact.GetNormal();
    float inverseMassA = bodyA->GetInverseMass();
    float inverseMassB = bodyB->GetInverseMass();
    Mat3 inverseInertiaA = bodyA->GetWorldInverseInertiaTensor();
    Mat3 inverseInertiaB = bodyB->GetWorldInverseInertiaTensor();

    // Solve normal impulse for each contact point.
    for (int i = 0; i < contact.GetPointCount(); ++i) {
        ContactPoint& point = contact.GetPoint(i);

        Vec3 rA = point.position - bodyA->GetPosition();
        Vec3 rB = point.position - bodyB->GetPosition();

        Vec3 velocityA =
            bodyA->GetLinearVelocity() +
            bodyA->GetAngularVelocity().Cross(rA);

        Vec3 velocityB =
            bodyB->GetLinearVelocity() +
            bodyB->GetAngularVelocity().Cross(rB);

        Vec3 relativeVelocity = velocityB - velocityA;
        float normalVelocity = relativeVelocity.Dot(normal);

        if (point.normalImpulse == 0.0f &&
            normalVelocity < -m_restitutionThreshold) {
            point.tangentImpulse = 0.0f;
        }

        float targetVelocity = 0.0f;

        if (normalVelocity < -m_restitutionThreshold)
            targetVelocity = -contact.GetRestitution() * normalVelocity;

        Vec3 angularTermA =
            inverseInertiaA * rA.Cross(normal);

        Vec3 angularTermB =
            inverseInertiaB * rB.Cross(normal);

        float denominator =
            inverseMassA +
            inverseMassB +
            normal.Dot(angularTermA.Cross(rA)) +
            normal.Dot(angularTermB.Cross(rB));

        if (denominator <= 0.000001f)
            continue;

        float impulseMagnitude =
            -(normalVelocity - targetVelocity) / denominator;

        float oldImpulse = point.normalImpulse;

        float newImpulse = std::max(
            0.0f,
            oldImpulse + impulseMagnitude
        );

        float deltaImpulse = newImpulse - oldImpulse;

        if (std::abs(deltaImpulse) <= 0.000001f)
            continue;

        Vec3 impulse = normal * deltaImpulse;

        bodyA->SetLinearVelocity(
            bodyA->GetLinearVelocity() - impulse * inverseMassA, false
        );

        bodyB->SetLinearVelocity(
            bodyB->GetLinearVelocity() + impulse * inverseMassB, false
        );

        bodyA->SetAngularVelocity(
            bodyA->GetAngularVelocity() -
            inverseInertiaA * rA.Cross(impulse), false
        );

        bodyB->SetAngularVelocity(
            bodyB->GetAngularVelocity() +
            inverseInertiaB * rB.Cross(impulse), false
        );

        Logger::Debug(
            "rA.Cross(normal): " +
            std::to_string(rA.Cross(normal).x) + ',' +
            std::to_string(rA.Cross(normal).y) + ',' +
            std::to_string(rA.Cross(normal).z) +
            ", Normal Velocity: " + std::to_string(normalVelocity) +
            ", Target Velocity: " + std::to_string(targetVelocity) +
            ", Angular Velocity A: " +
            std::to_string(bodyA->GetAngularVelocity().x) + ',' +
            std::to_string(bodyA->GetAngularVelocity().y) + ',' +
            std::to_string(bodyA->GetAngularVelocity().z) +
            ", Linear Velocity A: " +
            std::to_string(bodyA->GetLinearVelocity().x) + ',' +
            std::to_string(bodyA->GetLinearVelocity().y) + ',' +
            std::to_string(bodyA->GetLinearVelocity().z)
        );

        point.normalImpulse = newImpulse;
    }
}

void Solver::SolveFriction(Contact& contact) {
    RigidBody* bodyA = contact.GetBodyA();
    RigidBody* bodyB = contact.GetBodyB();

    if (!bodyA || !bodyB)
        return;

    Vec3 normal = contact.GetNormal();
    float inverseMassA = bodyA->GetInverseMass();
    float inverseMassB = bodyB->GetInverseMass();
    Mat3 inverseInertiaA = bodyA->GetWorldInverseInertiaTensor();
    Mat3 inverseInertiaB = bodyB->GetWorldInverseInertiaTensor();

    for (int i = 0; i < contact.GetPointCount(); ++i) {
        ContactPoint& point = contact.GetPoint(i);

        if (point.normalImpulse <= 0.0f)
            continue;

        Vec3 rA = point.position - bodyA->GetPosition();
        Vec3 rB = point.position - bodyB->GetPosition();

        Vec3 velocityA =
            bodyA->GetLinearVelocity() +
            bodyA->GetAngularVelocity().Cross(rA);

        Vec3 velocityB =
            bodyB->GetLinearVelocity() +
            bodyB->GetAngularVelocity().Cross(rB);

        Vec3 relativeVelocity = velocityB - velocityA;

        Vec3 tangentVelocity =
            relativeVelocity -
            normal * relativeVelocity.Dot(normal);

        float tangentLength = tangentVelocity.Length();

        if (tangentLength <= 0.000001f)
            continue;

        Vec3 tangent = tangentVelocity / tangentLength;

        Vec3 angularTermA =
            inverseInertiaA * rA.Cross(tangent);

        Vec3 angularTermB =
            inverseInertiaB * rB.Cross(tangent);

        float denominator =
            inverseMassA +
            inverseMassB +
            tangent.Dot(angularTermA.Cross(rA)) +
            tangent.Dot(angularTermB.Cross(rB));

        if (denominator <= 0.000001f)
            continue;

        float impulseMagnitude =
            -relativeVelocity.Dot(tangent) / denominator;

        float oldImpulse = point.tangentImpulse;
        float maxFrictionImpulse =
            contact.GetFriction() * point.normalImpulse;

        float newImpulse = oldImpulse + impulseMagnitude;

        if (newImpulse < -maxFrictionImpulse)
            newImpulse = -maxFrictionImpulse;
        else if (newImpulse > maxFrictionImpulse)
            newImpulse = maxFrictionImpulse;

        float deltaImpulse = newImpulse - oldImpulse;

        if (std::abs(deltaImpulse) <= 0.000001f)
            continue;

        Vec3 impulse = tangent * deltaImpulse;

        bodyA->SetLinearVelocity(
            bodyA->GetLinearVelocity() - impulse * inverseMassA, false
        );

        bodyB->SetLinearVelocity(
            bodyB->GetLinearVelocity() + impulse * inverseMassB, false
        );

        bodyA->SetAngularVelocity(
            bodyA->GetAngularVelocity() -
            inverseInertiaA * rA.Cross(impulse), false
        );

        bodyB->SetAngularVelocity(
            bodyB->GetAngularVelocity() +
            inverseInertiaB * rB.Cross(impulse), false
        );

        point.tangentImpulse = newImpulse;
    }
}