#include "Solver.h"
#include "RigidBody.h"
#include <algorithm>
#include <cmath>
#include <string>
#include "../Core/Debug/Logger.h"

Solver::Solver()
    : m_penetrationSlop(0.001f),
    m_correctionPercent(0.2f),
    m_velocityIterations(8),
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

    for (int i = 0; i < contact.GetPointCount(); ++i)
        maxPenetration = std::max(maxPenetration, contact.GetPoint(i).penetration);

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

    bodyA->SetPosition(bodyA->GetPosition() - correction * inverseMassA);
    bodyB->SetPosition(bodyB->GetPosition() + correction * inverseMassB);
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

    Vec3 initialVelocityA = bodyA->GetLinearVelocity();
    Vec3 initialAngularVelocityA = bodyA->GetAngularVelocity();

    float totalImpulse = 0.0f;
    float maxImpulse = 0.0f;
    float minNormalVelocity = INFINITY;
    float maxNormalVelocity = -INFINITY;
    int appliedImpulseCount = 0;

    for (int i = 0; i < contact.GetPointCount(); ++i) {
        ContactPoint& point = contact.GetPoint(i);
        Logger::Debug(
            "[SOLVER] " "Point" + std::to_string(i) + ": " + "(" +
            std::to_string(point.position.x) + "," +
            std::to_string(point.position.y) + "," +
            std::to_string(point.position.z) + ")"
        );

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

        minNormalVelocity = std::min(minNormalVelocity, normalVelocity);
        maxNormalVelocity = std::max(maxNormalVelocity, normalVelocity);

        if (point.normalImpulse == 0.0f &&
            normalVelocity < -m_restitutionThreshold)
            point.tangentImpulse = 0.0f;

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
        float newImpulse =
            std::max(0.0f, oldImpulse + impulseMagnitude);

        float deltaImpulse =
            newImpulse - oldImpulse;

        if (std::abs(deltaImpulse) <= 0.000001f)
            continue;

        Vec3 impulse = normal * deltaImpulse;

        bodyA->SetLinearVelocity(
            bodyA->GetLinearVelocity() -
            impulse * inverseMassA
        );

        bodyB->SetLinearVelocity(
            bodyB->GetLinearVelocity() +
            impulse * inverseMassB
        );

        bodyA->SetAngularVelocity(
            bodyA->GetAngularVelocity() -
            inverseInertiaA * rA.Cross(impulse)
        );

        bodyB->SetAngularVelocity(
            bodyB->GetAngularVelocity() +
            inverseInertiaB * rB.Cross(impulse)
        );

        point.normalImpulse = newImpulse;

        totalImpulse += std::fabs(deltaImpulse);
        maxImpulse = std::max(maxImpulse, std::fabs(deltaImpulse));
        ++appliedImpulseCount;
    }

    Vec3 finalVelocityA = bodyA->GetLinearVelocity();
    Vec3 finalAngularVelocityA = bodyA->GetAngularVelocity();

    Logger::Debug(
        "[SOLVER] "
        "N=(" +
        std::to_string(normal.x) + "," +
        std::to_string(normal.y) + "," +
        std::to_string(normal.z) + ") "
        "NV=(" +
        std::to_string(minNormalVelocity) + "," +
        std::to_string(maxNormalVelocity) + ") "
        "IMP=" +
        std::to_string(totalImpulse) +
        " MAX=" +
        std::to_string(maxImpulse) +
        " CNT=" +
        std::to_string(appliedImpulseCount) +
        " V=(" +
        std::to_string(initialVelocityA.x) + "," +
        std::to_string(initialVelocityA.y) + "," +
        std::to_string(initialVelocityA.z) +
        ")->(" +
        std::to_string(finalVelocityA.x) + "," +
        std::to_string(finalVelocityA.y) + "," +
        std::to_string(finalVelocityA.z) +
        ") "
        "W=(" +
        std::to_string(initialAngularVelocityA.x) + "," +
        std::to_string(initialAngularVelocityA.y) + "," +
        std::to_string(initialAngularVelocityA.z) +
        ")->(" +
        std::to_string(finalAngularVelocityA.x) + "," +
        std::to_string(finalAngularVelocityA.y) + "," +
        std::to_string(finalAngularVelocityA.z) +
        ")"
    );
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

        float newImpulse =
            oldImpulse + impulseMagnitude;

        if (newImpulse > maxFrictionImpulse)
            newImpulse = maxFrictionImpulse;
        else if (newImpulse < -maxFrictionImpulse)
            newImpulse = -maxFrictionImpulse;

        float deltaImpulse =
            newImpulse - oldImpulse;

        Logger::Debug(
            "[IMPULSE] "
            "P" + std::to_string(i) +
            " NV=" + std::to_string(relativeVelocity.Dot(normal)) +
            " DEN=" + std::to_string(denominator) +
            " CALC=" + std::to_string(impulseMagnitude) +
            " OLD=" + std::to_string(oldImpulse) +
            " NEW=" + std::to_string(newImpulse) +
            " DELTA=" + std::to_string(deltaImpulse)
        );

        if (std::abs(deltaImpulse) <= 0.000001f)
            continue;

        Vec3 impulse = tangent * deltaImpulse;

        bodyA->SetLinearVelocity(
            bodyA->GetLinearVelocity() -
            impulse * inverseMassA
        );

        bodyB->SetLinearVelocity(
            bodyB->GetLinearVelocity() +
            impulse * inverseMassB
        );

        bodyA->SetAngularVelocity(
            bodyA->GetAngularVelocity() -
            inverseInertiaA * rA.Cross(impulse)
        );

        bodyB->SetAngularVelocity(
            bodyB->GetAngularVelocity() +
            inverseInertiaB * rB.Cross(impulse)
        );

        point.tangentImpulse = newImpulse;
    }
}