// Physics/Solver.cpp

#include "Solver.h"
#include "RigidBody.h"
#include "../Core/Debug/Logger.h"

#include <algorithm>
#include <cmath>

namespace {
    constexpr float Epsilon = 0.000001f;
    constexpr float ImpulseTolerance = 0.00001f;
    constexpr float ResidualTolerance = 0.0001f;
    constexpr int MaxBlockSize = Contact::MaxPoints;

    bool SolveLinearSystem(
        const float matrix[MaxBlockSize][MaxBlockSize],
        const float rhs[MaxBlockSize],
        float result[MaxBlockSize],
        int size
    ) {
        if (size <= 0)
            return false;

        float augmented[MaxBlockSize][MaxBlockSize + 1] = {};

        for (int row = 0; row < size; ++row) {
            for (int column = 0; column < size; ++column)
                augmented[row][column] =
                matrix[row][column];

            augmented[row][size] = rhs[row];
        }

        for (int column = 0; column < size; ++column) {
            int pivotRow = column;
            float pivotMagnitude =
                std::fabs(augmented[column][column]);

            for (int row = column + 1; row < size; ++row) {
                float magnitude =
                    std::fabs(augmented[row][column]);

                if (magnitude > pivotMagnitude) {
                    pivotMagnitude = magnitude;
                    pivotRow = row;
                }
            }

            if (pivotMagnitude <= Epsilon)
                return false;

            if (pivotRow != column) {
                for (int j = column; j <= size; ++j)
                    std::swap(
                        augmented[column][j],
                        augmented[pivotRow][j]
                    );
            }

            float pivot =
                augmented[column][column];

            for (int j = column; j <= size; ++j)
                augmented[column][j] /= pivot;

            for (int row = 0; row < size; ++row) {
                if (row == column)
                    continue;

                float factor =
                    augmented[row][column];

                if (std::fabs(factor) <= Epsilon)
                    continue;

                for (int j = column; j <= size; ++j)
                    augmented[row][j] -=
                    factor * augmented[column][j];
            }
        }

        for (int i = 0; i < size; ++i)
            result[i] = augmented[i][size];

        return true;
    }
}

Solver::Solver()
    : m_penetrationSlop(0.005f),
    m_correctionPercent(0.2f),
    m_velocityIterations(16),
    m_positionIterations(4),
    m_restitutionThreshold(1.0f) {}

void Solver::SetPenetrationSlop(float slop) {
    m_penetrationSlop =
        std::max(0.0f, slop);
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
    m_velocityIterations =
        std::max(1, iterations);
}

int Solver::GetVelocityIterations() const {
    return m_velocityIterations;
}

void Solver::SetPositionIterations(int iterations) {
    m_positionIterations =
        std::max(1, iterations);
}

int Solver::GetPositionIterations() const {
    return m_positionIterations;
}

void Solver::SetRestitutionThreshold(float threshold) {
    m_restitutionThreshold =
        std::max(0.0f, threshold);
}

float Solver::GetRestitutionThreshold() const {
    return m_restitutionThreshold;
}

void Solver::PrepareVelocityBias(Contact& contact) {
    RigidBody* bodyA =
        contact.GetBodyA();

    RigidBody* bodyB =
        contact.GetBodyB();

    if (!bodyA || !bodyB)
        return;

    const Vec3 normal =
        contact.GetNormal();

    for (int i = 0; i < contact.GetPointCount(); ++i) {
        ContactPoint& point =
            contact.GetPoint(i);

        const Vec3 rA =
            point.position -
            bodyA->GetPosition();

        const Vec3 rB =
            point.position -
            bodyB->GetPosition();

        const Vec3 velocityA =
            bodyA->GetLinearVelocity() +
            bodyA->GetAngularVelocity().Cross(rA);

        const Vec3 velocityB =
            bodyB->GetLinearVelocity() +
            bodyB->GetAngularVelocity().Cross(rB);

        const float normalVelocity =
            (velocityB - velocityA).Dot(normal);

        point.velocityBias = 0.0f;

        if (normalVelocity < -m_restitutionThreshold) {
            point.velocityBias =
                -contact.GetRestitution() *
                normalVelocity;
        }
    }
}

void Solver::WarmStart(Contact& contact) {
    RigidBody* bodyA =
        contact.GetBodyA();

    RigidBody* bodyB =
        contact.GetBodyB();

    if (!bodyA || !bodyB)
        return;

    const Vec3 normal =
        contact.GetNormal();

    const Vec3 tangent1 =
        contact.GetTangent1();

    const Vec3 tangent2 =
        contact.GetTangent2();

    const float inverseMassA =
        bodyA->GetInverseMass();

    const float inverseMassB =
        bodyB->GetInverseMass();

    const Mat3 inverseInertiaA =
        bodyA->GetWorldInverseInertiaTensor();

    const Mat3 inverseInertiaB =
        bodyB->GetWorldInverseInertiaTensor();

    for (int i = 0; i < contact.GetPointCount(); ++i) {
        ContactPoint& point =
            contact.GetPoint(i);

        Vec3 rA =
            point.position -
            bodyA->GetPosition();

        Vec3 rB =
            point.position -
            bodyB->GetPosition();

        Vec3 impulse =
            normal * point.normalImpulse +
            tangent1 * point.tangentImpulse1 +
            tangent2 * point.tangentImpulse2;

        if (impulse.LengthSquared() <= Epsilon)
            continue;

        bodyA->SetLinearVelocity(
            bodyA->GetLinearVelocity() -
            impulse * inverseMassA,
            false
        );

        bodyB->SetLinearVelocity(
            bodyB->GetLinearVelocity() +
            impulse * inverseMassB,
            false
        );

        bodyA->SetAngularVelocity(
            bodyA->GetAngularVelocity() -
            inverseInertiaA *
            rA.Cross(impulse),
            false
        );

        bodyB->SetAngularVelocity(
            bodyB->GetAngularVelocity() +
            inverseInertiaB *
            rB.Cross(impulse),
            false
        );
    }
}

void Solver::SolvePosition(Contact& contact) {
    /*
        Nonlinear Gauss-Seidel(NGS) 위치 보정.

        예전 구현은 매니폴드 전체에서 "가장 깊은 침투량" 하나만 뽑아서
        두 바디를 법선 방향으로 한 번 밀어내는 게 전부였다. 이건:

          1) 회전을 전혀 교정하지 못한다 (박스가 살짝 기울어진 채로
             박혀 있어도, 이 함수는 위치만 옮기고 자세는 절대 안 바꾼다.
             속도 솔버도 "정지 상태에서" 자세 오차를 되돌리는 항이
             없으니(반발계수 바이어스는 접근 속도가 있을 때만 작동),
             한 번 생긴 기울어짐은 영원히 남거나 계속 자란다.)
          2) 여러 접촉점의 침투 깊이가 서로 다를 때(=바디가 비대칭으로
             박혀 있을 때) 그 비대칭을 무시하고 똑같이 밀어버려서,
             다음 프레임에 또 비대칭 침투가 재생산된다.

        아래 구현은 각 접촉점마다:
          - 로컬 앵커를 "현재" 위치/자세로 다시 투영해서 실제 분리량을
            구하고 (한 Contact 안에서 여러 점을 처리하는 동안 자세가
            바뀌어도 계속 최신 상태를 반영한다),
          - 선형 + 회전 성분을 모두 포함한 유효질량으로 pseudo-impulse를
            계산해서,
          - 속도에는 전혀 손대지 않고 위치와 자세를 직접 수정한다.

        PhysicsWorld는 이 함수를 프레임당 여러 번(포지션 이터레이션) 호출해서
        점진적으로 수렴시킨다.
    */

    RigidBody* bodyA =
        contact.GetBodyA();

    RigidBody* bodyB =
        contact.GetBodyB();

    if (!bodyA || !bodyB)
        return;

    const float inverseMassA =
        bodyA->GetInverseMass();

    const float inverseMassB =
        bodyB->GetInverseMass();

    if (inverseMassA + inverseMassB <= 0.0f)
        return;

    const Vec3 normal =
        contact.GetNormal();

    constexpr float MaxLinearCorrection = 0.2f;

    for (int i = 0; i < contact.GetPointCount(); ++i) {
        ContactPoint& point =
            contact.GetPoint(i);

        // 캐시된 rA/rB가 아니라, "지금" 자세로 다시 계산한다. 이게
        // NGS의 핵심이다 — 회전 보정이 누적될수록 앵커가 실제 표면
        // 위치를 계속 따라가야 한다.
        const Vec3 rA =
            bodyA->GetOrientation() *
            point.localAnchorA;

        const Vec3 rB =
            bodyB->GetOrientation() *
            point.localAnchorB;

        const Vec3 worldA =
            bodyA->GetPosition() + rA;

        const Vec3 worldB =
            bodyB->GetPosition() + rB;

        const float separation =
            (worldB - worldA).Dot(normal);

        const Mat3 inverseInertiaA =
            bodyA->GetWorldInverseInertiaTensor();

        const Mat3 inverseInertiaB =
            bodyB->GetWorldInverseInertiaTensor();

        const Vec3 angularA =
            inverseInertiaA * rA.Cross(normal);

        const Vec3 angularB =
            inverseInertiaB * rB.Cross(normal);

        float effectiveMass =
            inverseMassA + inverseMassB;

        effectiveMass +=
            normal.Dot(angularA.Cross(rA));

        effectiveMass +=
            normal.Dot(angularB.Cross(rB));

        if (effectiveMass <= Epsilon)
            continue;

        // 슬롭만큼은 허용, 한 번에 너무 크게 밀지 않도록 클램프.
        float correction =
            m_correctionPercent *
            (separation + m_penetrationSlop);

        correction =
            std::max(correction, -MaxLinearCorrection);

        correction =
            std::min(correction, 0.0f);

        if (correction >= 0.0f)
            continue;

        const float impulse =
            -correction / effectiveMass;

        const Vec3 P =
            normal * impulse;

        if (inverseMassA > 0.0f) {
            bodyA->SetPosition(
                bodyA->GetPosition() -
                P * inverseMassA
            );

            const Vec3 angularCorrectionA =
                inverseInertiaA *
                rA.Cross(-P);

            Quaternion orientationA =
                bodyA->GetOrientation();

            const Quaternion derivativeA =
                Quaternion(
                    0.0f,
                    angularCorrectionA.x,
                    angularCorrectionA.y,
                    angularCorrectionA.z
                ) * orientationA;

            orientationA.w += 0.5f * derivativeA.w;
            orientationA.x += 0.5f * derivativeA.x;
            orientationA.y += 0.5f * derivativeA.y;
            orientationA.z += 0.5f * derivativeA.z;

            orientationA.Normalize();
            bodyA->SetOrientation(orientationA);
        }

        if (inverseMassB > 0.0f) {
            bodyB->SetPosition(
                bodyB->GetPosition() +
                P * inverseMassB
            );

            const Vec3 angularCorrectionB =
                inverseInertiaB *
                rB.Cross(P);

            Quaternion orientationB =
                bodyB->GetOrientation();

            const Quaternion derivativeB =
                Quaternion(
                    0.0f,
                    angularCorrectionB.x,
                    angularCorrectionB.y,
                    angularCorrectionB.z
                ) * orientationB;

            orientationB.w += 0.5f * derivativeB.w;
            orientationB.x += 0.5f * derivativeB.x;
            orientationB.y += 0.5f * derivativeB.y;
            orientationB.z += 0.5f * derivativeB.z;

            orientationB.Normalize();
            bodyB->SetOrientation(orientationB);
        }
    }
}

void Solver::SolveVelocity(Contact& contact) {
    RigidBody* bodyA =
        contact.GetBodyA();

    RigidBody* bodyB =
        contact.GetBodyB();

    if (!bodyA || !bodyB)
        return;

    const int pointCount =
        contact.GetPointCount();

    if (pointCount <= 0)
        return;


    const Vec3 normal =
        contact.GetNormal();

    const float inverseMassA =
        bodyA->GetInverseMass();

    const float inverseMassB =
        bodyB->GetInverseMass();

    const Mat3 inverseInertiaA =
        bodyA->GetWorldInverseInertiaTensor();

    const Mat3 inverseInertiaB =
        bodyB->GetWorldInverseInertiaTensor();

    /*
        One contact point:
        Use ordinary Sequential Impulse / Gauss-Seidel.

        There is no reason to build a block system for a
        single constraint.
    */
    if (pointCount == 1) {
        ContactPoint& point =
            contact.GetPoint(0);

        const Vec3 rA =
            point.position -
            bodyA->GetPosition();

        const Vec3 rB =
            point.position -
            bodyB->GetPosition();

        const Vec3 velocityA =
            bodyA->GetLinearVelocity() +
            bodyA->GetAngularVelocity().Cross(rA);

        const Vec3 velocityB =
            bodyB->GetLinearVelocity() +
            bodyB->GetAngularVelocity().Cross(rB);

        const Vec3 relativeVelocity =
            velocityB - velocityA;

        const float normalVelocity =
            relativeVelocity.Dot(normal);

        // 반발 목표 속도는 이 프레임 시작 시 PrepareVelocityBias()가
        // 한 번만 계산해 둔 고정값을 쓴다 (매 반복 재계산 금지 — 이유는
        // ContactPoint::velocityBias 주석 참고).
        const float targetVelocity =
            point.velocityBias;

        const Vec3 angularA =
            inverseInertiaA *
            rA.Cross(normal);

        const Vec3 angularB =
            inverseInertiaB *
            rB.Cross(normal);

        float effectiveMass =
            inverseMassA +
            inverseMassB;

        effectiveMass +=
            normal.Dot(
                angularA.Cross(rA)
            );

        effectiveMass +=
            normal.Dot(
                angularB.Cross(rB)
            );

        if (effectiveMass <= Epsilon)
            return;

        float deltaImpulse =
            (targetVelocity -
                normalVelocity) /
            effectiveMass;


        const float oldImpulse =
            point.normalImpulse;

        float newImpulse =
            oldImpulse +
            deltaImpulse;

        if (newImpulse < 0.0f)
            newImpulse = 0.0f;

        deltaImpulse =
            newImpulse -
            oldImpulse;

        point.normalImpulse =
            newImpulse;

        if (std::fabs(deltaImpulse) <= Epsilon)
            return;

        const Vec3 impulse =
            normal * deltaImpulse;

        bodyA->SetLinearVelocity(
            bodyA->GetLinearVelocity() -
            impulse * inverseMassA,
            false
        );

        bodyB->SetLinearVelocity(
            bodyB->GetLinearVelocity() +
            impulse * inverseMassB,
            false
        );

        bodyA->SetAngularVelocity(
            bodyA->GetAngularVelocity() -
            inverseInertiaA *
            rA.Cross(impulse),
            false
        );

        bodyB->SetAngularVelocity(
            bodyB->GetAngularVelocity() +
            inverseInertiaB *
            rB.Cross(impulse),
            false
        );

        return;
    }

    /*
        Multiple contact points:
        Treat the manifold as one coupled constraint system.

        This is the important part for face-face and similar
        multi-point contacts.

        K * lambda = rhs

        lambda is solved simultaneously instead of solving
        C1, then C2, then C3, then C4 independently.
    */

    float effectiveMass[
        MaxBlockSize][MaxBlockSize] = {};

        float rhs[
            MaxBlockSize
        ] = {};

            float oldImpulse[
                MaxBlockSize
            ] = {};

                float newImpulse[
                    MaxBlockSize
                ] = {};

                    float normalVelocity[
                        MaxBlockSize
                    ] = {};

                        float targetVelocity[
                            MaxBlockSize
                        ] = {};

						Vec3 rA[MaxBlockSize];
						Vec3 rB[MaxBlockSize];

                            for (int i = 0; i < pointCount; ++i) {
                                ContactPoint& point =
                                    contact.GetPoint(i);

                                rA[i] =
                                    point.position -
                                    bodyA->GetPosition();

                                rB[i] =
                                    point.position -
                                    bodyB->GetPosition();

                                const Vec3 velocityA =
                                    bodyA->GetLinearVelocity() +
                                    bodyA->GetAngularVelocity().Cross(rA[i]);

                                const Vec3 velocityB =
                                    bodyB->GetLinearVelocity() +
                                    bodyB->GetAngularVelocity().Cross(rB[i]);

                                const Vec3 relativeVelocity =
                                    velocityB - velocityA;

                                normalVelocity[i] =
                                    relativeVelocity.Dot(normal);

                                // 고정된(프레임당 1회 계산) 반발 바이어스를 사용한다.
                                targetVelocity[i] =
                                    point.velocityBias;

                                oldImpulse[i] =
                                    point.normalImpulse;


                            }

                            for (int i = 0; i < pointCount; ++i) {
                                const Vec3 angularA =
                                    inverseInertiaA *
                                    rA[i].Cross(normal);

                                const Vec3 angularB =
                                    inverseInertiaB *
                                    rB[i].Cross(normal);

                                for (int j = 0; j < pointCount; ++j) {
                                    float value =
                                        inverseMassA +
                                        inverseMassB;

                                    value +=
                                        normal.Dot(
                                            angularA.Cross(rA[j])
                                        );

                                    value +=
                                        normal.Dot(
                                            angularB.Cross(rB[j])
                                        );

                                    effectiveMass[i][j] =
                                        value;
                                }
                            }

                            for (int i = 0; i < pointCount; ++i) {
                                rhs[i] =
                                    targetVelocity[i] -
                                    normalVelocity[i];

                                for (int j = 0; j < pointCount; ++j) {
                                    rhs[i] +=
                                        effectiveMass[i][j] *
                                        oldImpulse[j];
                                }
                            }

                            /*
                                Solve the unilateral contact problem.

                                For each possible active set:
                                    active   -> lambda >= 0, residual = 0
                                    inactive -> lambda = 0, residual >= 0

                                MaxPoints is 4, so at most 16 active sets exist.
                            */

                            bool solved = false;

                            const int subsetCount =
                                1 << pointCount;

                            for (int mask = 0;
                                mask < subsetCount && !solved;
                                ++mask) {

                                int activeIndices[
                                    MaxBlockSize
                                ];

                                int activeCount = 0;

                                for (int i = 0; i < pointCount; ++i) {
                                    if (mask & (1 << i))
                                        activeIndices[activeCount++] = i;
                                }

                                float subMatrix[
                                    MaxBlockSize][MaxBlockSize
                                    ] = {};

                                    float subRhs[
                                        MaxBlockSize
                                    ] = {};

                                        float subResult[
                                            MaxBlockSize
                                        ] = {};

                                            for (int row = 0;
                                                row < activeCount;
                                                ++row) {

                                                const int originalRow =
                                                    activeIndices[row];

                                                subRhs[row] =
                                                    rhs[originalRow];

                                                for (int column = 0;
                                                    column < activeCount;
                                                    ++column) {

                                                    const int originalColumn =
                                                        activeIndices[column];

                                                    subMatrix[row][column] =
                                                        effectiveMass[
                                                            originalRow
                                                        ][originalColumn];
                                                }
                                            }

                                            /*
                                                Empty active set is a valid solution when every
                                                contact constraint already satisfies its condition.
                                            */
                                            bool linearSystemSolved =
                                                activeCount == 0 ||
                                                SolveLinearSystem(
                                                    subMatrix,
                                                    subRhs,
                                                    subResult,
                                                    activeCount
                                                );

                                            if (!linearSystemSolved)
                                                continue;

                                            for (int i = 0; i < pointCount; ++i)
                                                newImpulse[i] = 0.0f;

                                            bool valid = true;

                                            for (int i = 0;
                                                i < activeCount;
                                                ++i) {

                                                const int originalIndex =
                                                    activeIndices[i];

                                                newImpulse[originalIndex] =
                                                    subResult[i];

                                                if (newImpulse[originalIndex] <
                                                    -ImpulseTolerance) {

                                                    valid = false;
                                                    break;
                                                }
                                            }

                                            if (!valid)
                                                continue;

                                            for (int i = 0; i < pointCount; ++i) {
                                                if (newImpulse[i] < 0.0f)
                                                    newImpulse[i] = 0.0f;
                                            }

                                            for (int i = 0; i < pointCount; ++i) {
                                                float residual =
                                                    -rhs[i];

                                                for (int j = 0;
                                                    j < pointCount;
                                                    ++j) {

                                                    residual +=
                                                        effectiveMass[i][j] *
                                                        newImpulse[j];
                                                }

                                                if (mask & (1 << i)) {
                                                    if (std::fabs(residual) >
                                                        ResidualTolerance) {

                                                        valid = false;
                                                        break;
                                                    }
                                                }
                                                else {
                                                    if (residual <
                                                        -ImpulseTolerance) {

                                                        valid = false;
                                                        break;
                                                    }
                                                }
                                            }

                                            if (valid)
                                                solved = true;
                            }

                            /*
                                Numerical fallback.

                                A block system can become nearly singular in unusual
                                configurations. Do not throw away the entire contact
                                response in that case.

                                Fall back to ordinary GS using the current accumulated
                                impulses.
                            */
                            if (!solved) {
                                for (int iteration = 0;
                                    iteration < pointCount;
                                    ++iteration) {

                                    for (int i = 0;
                                        i < pointCount;
                                        ++i) {

                                        const Vec3 angularA =
                                            inverseInertiaA *
                                            rA[i].Cross(normal);

                                        const Vec3 angularB =
                                            inverseInertiaB *
                                            rB[i].Cross(normal);

                                        float denominator =
                                            inverseMassA +
                                            inverseMassB;

                                        denominator +=
                                            normal.Dot(
                                                angularA.Cross(rA[i])
                                            );

                                        denominator +=
                                            normal.Dot(
                                                angularB.Cross(rB[i])
                                            );

                                        if (denominator <= Epsilon)
                                            continue;

                                        const Vec3 velocityA =
                                            bodyA->GetLinearVelocity() +
                                            bodyA->GetAngularVelocity().Cross(rA[i]);

                                        const Vec3 velocityB =
                                            bodyB->GetLinearVelocity() +
                                            bodyB->GetAngularVelocity().Cross(rB[i]);

                                        const float velocity =
                                            (velocityB - velocityA).Dot(normal);

                                        // 여기도 동일하게 고정 바이어스 사용.
                                        const float target =
                                            contact.GetPoint(i).velocityBias;

                                        const float old =
                                            contact.GetPoint(i).normalImpulse;

                                        float next =
                                            old +
                                            (target - velocity) /
                                            denominator;

                                        if (next < 0.0f)
                                            next = 0.0f;

                                        const float delta =
                                            next - old;

                                        contact.GetPoint(i).normalImpulse =
                                            next;

                                        if (std::fabs(delta) <= Epsilon)
                                            continue;

                                        const Vec3 impulse =
                                            normal * delta;

                                        bodyA->SetLinearVelocity(
                                            bodyA->GetLinearVelocity() -
                                            impulse * inverseMassA,
                                            false
                                        );

                                        bodyB->SetLinearVelocity(
                                            bodyB->GetLinearVelocity() +
                                            impulse * inverseMassB,
                                            false
                                        );

                                        bodyA->SetAngularVelocity(
                                            bodyA->GetAngularVelocity() -
                                            inverseInertiaA *
                                            rA[i].Cross(impulse),
                                            false
                                        );

                                        bodyB->SetAngularVelocity(
                                            bodyB->GetAngularVelocity() +
                                            inverseInertiaB *
                                            rB[i].Cross(impulse),
                                            false
                                        );
                                    }
                                }

                                return;
                            }

                            /*
                                Apply only the difference between the newly solved
                                manifold impulses and the accumulated impulses.
                            */

                            for (int i = 0; i < pointCount; ++i) {
                                ContactPoint& point =
                                    contact.GetPoint(i);

                                const float deltaImpulse =
                                    newImpulse[i] -
                                    oldImpulse[i];

                                point.normalImpulse =
                                    newImpulse[i];

                                if (std::fabs(deltaImpulse) <= Epsilon)
                                    continue;

                                const Vec3 impulse =
                                    normal * deltaImpulse;

                                bodyA->SetLinearVelocity(
                                    bodyA->GetLinearVelocity() -
                                    impulse * inverseMassA,
                                    false
                                );

                                bodyB->SetLinearVelocity(
                                    bodyB->GetLinearVelocity() +
                                    impulse * inverseMassB,
                                    false
                                );

                                bodyA->SetAngularVelocity(
                                    bodyA->GetAngularVelocity() -
                                    inverseInertiaA *
                                    rA[i].Cross(impulse),
                                    false
                                );

                                bodyB->SetAngularVelocity(
                                    bodyB->GetAngularVelocity() +
                                    inverseInertiaB *
                                    rB[i].Cross(impulse),
                                    false
                                );
                            }
}

void Solver::SolveFriction(Contact& contact) {
    RigidBody* bodyA =
        contact.GetBodyA();

    RigidBody* bodyB =
        contact.GetBodyB();

    if (!bodyA || !bodyB)
        return;

    const Vec3 tangent1 =
        contact.GetTangent1();

    const Vec3 tangent2 =
        contact.GetTangent2();

    const float inverseMassA =
        bodyA->GetInverseMass();

    const float inverseMassB =
        bodyB->GetInverseMass();

    const Mat3 inverseInertiaA =
        bodyA->GetWorldInverseInertiaTensor();

    const Mat3 inverseInertiaB =
        bodyB->GetWorldInverseInertiaTensor();

    for (int i = 0; i < contact.GetPointCount(); ++i) {
        ContactPoint& point =
            contact.GetPoint(i);

        const Vec3 rA =
            point.position -
            bodyA->GetPosition();

        const Vec3 rB =
            point.position -
            bodyB->GetPosition();

        const Vec3 velocityA =
            bodyA->GetLinearVelocity() +
            bodyA->GetAngularVelocity().Cross(rA);

        const Vec3 velocityB =
            bodyB->GetLinearVelocity() +
            bodyB->GetAngularVelocity().Cross(rB);

        const Vec3 relativeVelocity =
            velocityB - velocityA;

        const float velocityT1 =
            relativeVelocity.Dot(tangent1);

        const float velocityT2 =
            relativeVelocity.Dot(tangent2);

        const Vec3 angularA1 =
            inverseInertiaA *
            rA.Cross(tangent1);

        const Vec3 angularB1 =
            inverseInertiaB *
            rB.Cross(tangent1);

        const Vec3 angularA2 =
            inverseInertiaA *
            rA.Cross(tangent2);

        const Vec3 angularB2 =
            inverseInertiaB *
            rB.Cross(tangent2);

        const float k11 =
            inverseMassA +
            inverseMassB +
            tangent1.Dot(
                angularA1.Cross(rA)
            ) +
            tangent1.Dot(
                angularB1.Cross(rB)
            );

        const float k22 =
            inverseMassA +
            inverseMassB +
            tangent2.Dot(
                angularA2.Cross(rA)
            ) +
            tangent2.Dot(
                angularB2.Cross(rB)
            );

        const float k12 =
            tangent1.Dot(
                angularA2.Cross(rA)
            ) +
            tangent1.Dot(
                angularB2.Cross(rB)
            );

        const float k21 =
            tangent2.Dot(
                angularA1.Cross(rA)
            ) +
            tangent2.Dot(
                angularB1.Cross(rB)
            );

        const float determinant =
            k11 * k22 -
            k12 * k21;

        if (std::fabs(determinant) <= Epsilon)
            continue;

        const float oldImpulse1 =
            point.tangentImpulse1;

        const float oldImpulse2 =
            point.tangentImpulse2;

        const float rhs1 =
            -velocityT1 +
            k11 * oldImpulse1 +
            k12 * oldImpulse2;

        const float rhs2 =
            -velocityT2 +
            k21 * oldImpulse1 +
            k22 * oldImpulse2;

        float deltaImpulse1 =
            (rhs1 * k22 -
                k12 * rhs2) /
            determinant;

        float deltaImpulse2 =
            (k11 * rhs2 -
                rhs1 * k21) /
            determinant;

        float newImpulse1 =
            oldImpulse1 +
            deltaImpulse1;

        float newImpulse2 =
            oldImpulse2 +
            deltaImpulse2;

        const float maxFrictionImpulse =
            contact.GetFriction() *
            point.normalImpulse;

        const float impulseLength =
            std::sqrt(
                newImpulse1 * newImpulse1 +
                newImpulse2 * newImpulse2
            );

        if (impulseLength >
            maxFrictionImpulse) {

            if (impulseLength > Epsilon) {
                const float scale =
                    maxFrictionImpulse /
                    impulseLength;

                newImpulse1 *= scale;
                newImpulse2 *= scale;
            }
            else {
                newImpulse1 = 0.0f;
                newImpulse2 = 0.0f;
            }
        }

        deltaImpulse1 =
            newImpulse1 -
            oldImpulse1;

        deltaImpulse2 =
            newImpulse2 -
            oldImpulse2;

        point.tangentImpulse1 =
            newImpulse1;

        point.tangentImpulse2 =
            newImpulse2;

        if (std::fabs(deltaImpulse1) <= Epsilon &&
            std::fabs(deltaImpulse2) <= Epsilon) {
            continue;
        }

        const Vec3 impulse =
            tangent1 * deltaImpulse1 +
            tangent2 * deltaImpulse2;

        bodyA->SetLinearVelocity(
            bodyA->GetLinearVelocity() -
            impulse * inverseMassA,
            false
        );

        bodyB->SetLinearVelocity(
            bodyB->GetLinearVelocity() +
            impulse * inverseMassB,
            false
        );

        bodyA->SetAngularVelocity(
            bodyA->GetAngularVelocity() -
            inverseInertiaA *
            rA.Cross(impulse),
            false
        );

        bodyB->SetAngularVelocity(
            bodyB->GetAngularVelocity() +
            inverseInertiaB *
            rB.Cross(impulse),
            false
        );
    }
}