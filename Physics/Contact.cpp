#include "Contact.h"

#include "RigidBody.h"

#include <cmath>

Contact::Contact()
    : m_bodyA(nullptr),
    m_bodyB(nullptr),
    m_normal(0.0f, 1.0f, 0.0f),
    m_tangent1(1.0f, 0.0f, 0.0f),
    m_tangent2(0.0f, 0.0f, -1.0f),
    m_pointCount(0),
    m_restitution(0.0f),
    m_friction(0.5f) {}

void Contact::SetBodies(
    RigidBody* bodyA,
    RigidBody* bodyB
) {
    m_bodyA = bodyA;
    m_bodyB = bodyB;
}

RigidBody* Contact::GetBodyA() {
    return m_bodyA;
}

const RigidBody* Contact::GetBodyA() const {
    return m_bodyA;
}

RigidBody* Contact::GetBodyB() {
    return m_bodyB;
}

const RigidBody* Contact::GetBodyB() const {
    return m_bodyB;
}

void Contact::SetNormal(const Vec3& normal) {
    m_normal = normal.Normalized();
    BuildTangentBasis();
}

const Vec3& Contact::GetNormal() const {
    return m_normal;
}

const Vec3& Contact::GetTangent1() const {
    return m_tangent1;
}

const Vec3& Contact::GetTangent2() const {
    return m_tangent2;
}

void Contact::BuildTangentBasis() {
    Vec3 reference;

    if (std::fabs(m_normal.x) < 0.9f)
        reference = Vec3(1.0f, 0.0f, 0.0f);
    else
        reference = Vec3(0.0f, 0.0f, 1.0f);

    m_tangent1 =
        reference.Cross(m_normal).Normalized();

    m_tangent2 =
        m_normal.Cross(m_tangent1).Normalized();
}

void Contact::SetRestitution(float restitution) {
    if (restitution < 0.0f)
        restitution = 0.0f;
    else if (restitution > 1.0f)
        restitution = 1.0f;

    m_restitution = restitution;
}

float Contact::GetRestitution() const {
    return m_restitution;
}

void Contact::SetFriction(float friction) {
    m_friction = friction > 0.0f ? friction : 0.0f;
}

float Contact::GetFriction() const {
    return m_friction;
}

void Contact::AddPoint(
    const Vec3& pointOnA,
    const Vec3& pointOnB,
    float penetration
) {
    if (m_pointCount >= MaxPoints)
        return;

    ContactPoint& point = m_points[m_pointCount];

    // 예전에는 "참조면에 투영된 점 하나"만 A/B 양쪽 앵커 계산에 같이
    // 썼는데, 그러면 두 바디의 로컬 앵커가 항상 같은 물리적 지점을
    // 가리키게 되어(둘 다 그 한 점에서 시작) NGS가 재투영으로 구하는
    // 분리량이 언제나 0 근처로만 나온다 — 실제 침투 깊이(penetration)
    // 와 완전히 따로 노는 값이 되어 위치 솔버가 사실상 아무것도
    // 고치지 못했다. pointOnA/pointOnB는 각 바디의 "진짜" 표면 위의
    // 점(둘 사이가 정확히 penetration만큼 떨어져 있음)이어야 한다.
    point.position =
        (pointOnA + pointOnB) * 0.5f;

    point.penetration =
        penetration > 0.0f ? penetration : 0.0f;

    point.normalImpulse = 0.0f;
    point.tangentImpulse1 = 0.0f;
    point.tangentImpulse2 = 0.0f;

    if (m_bodyA) {
        point.localAnchorA =
            m_bodyA->GetOrientation().Conjugate() *
            (pointOnA - m_bodyA->GetPosition());
    }

    if (m_bodyB) {
        point.localAnchorB =
            m_bodyB->GetOrientation().Conjugate() *
            (pointOnB - m_bodyB->GetPosition());
    }

    ++m_pointCount;
}

int Contact::GetPointCount() const {
    return m_pointCount;
}

ContactPoint& Contact::GetPoint(int index) {
    return m_points[index];
}

const ContactPoint& Contact::GetPoint(int index) const {
    return m_points[index];
}

void Contact::ClearPoints() {
    m_pointCount = 0;
}

void Contact::WarmStartFrom(const Contact& previous) {
    if (m_bodyA != previous.m_bodyA ||
        m_bodyB != previous.m_bodyB) {
        return;
    }

    if (m_normal.Dot(previous.m_normal) < 0.95f)
        return;

    constexpr float MatchDistanceSq =
        0.05f * 0.05f;

    bool used[MaxPoints] = {
        false,
        false,
        false,
        false
    };

    for (int i = 0; i < m_pointCount; ++i) {
        ContactPoint& current =
            m_points[i];

        int bestIndex = -1;
        float bestDistanceSq =
            MatchDistanceSq;

        for (int j = 0;
            j < previous.m_pointCount;
            ++j) {

            if (used[j])
                continue;

            const ContactPoint& old =
                previous.m_points[j];

            Vec3 difference =
                current.position -
                old.position;

            float distanceSq =
                difference.LengthSquared();

            if (distanceSq < bestDistanceSq) {
                bestDistanceSq = distanceSq;
                bestIndex = j;
            }
        }

        if (bestIndex < 0)
            continue;

        const ContactPoint& old =
            previous.m_points[bestIndex];

        current.normalImpulse =
            old.normalImpulse;

        current.tangentImpulse1 =
            old.tangentImpulse1;

        current.tangentImpulse2 =
            old.tangentImpulse2;

        used[bestIndex] = true;
    }
}