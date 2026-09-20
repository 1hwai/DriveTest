#pragma once

#include "../Core/Math/Vec3.h"

class RigidBody;

struct ContactPoint {
    Vec3 position;
    float penetration;

    // 바디 로컬 프레임 기준 앵커. 위치 솔버(NGS)가 여러 번 반복하며
    // "현재" 위치/자세로 접촉점을 다시 투영할 수 있게 해준다.
    // (world position만 캐싱해두면 바디가 회전할 때 실제 접촉점을
    //  따라가지 못해서 회전 오차를 절대 교정할 수 없다.)
    Vec3 localAnchorA;
    Vec3 localAnchorB;

    float normalImpulse;
    float tangentImpulse1;
    float tangentImpulse2;

    // 이번 프레임의 반발 목표 속도. 프레임당 딱 한 번, 워밍스타트 전의
    // "순수 충돌 접근 속도"로만 계산되어 그 프레임의 모든 속도 반복
    // (SolveVelocity가 여러 번 불림) 동안 고정된 값으로 쓰여야 한다.
    // 매 반복마다 "현재" 속도로 다시 계산하면, 첫 반복에서 튕겨나가는
    // 임펄스가 걸려 속도가 양수(분리 중)로 바뀌는 순간 다음 반복들이
    // "이미 분리 중이니 목표=0"이라고 판단해 그 임펄스를 도로 깎아버려서
    // 반발계수가 사실상 항상 무시되는 버그가 생긴다.
    float velocityBias;

    ContactPoint()
        : position(0.0f, 0.0f, 0.0f),
        penetration(0.0f),
        localAnchorA(0.0f, 0.0f, 0.0f),
        localAnchorB(0.0f, 0.0f, 0.0f),
        normalImpulse(0.0f),
        tangentImpulse1(0.0f),
        tangentImpulse2(0.0f),
        velocityBias(0.0f) {}
};

class Contact {
public:
    static constexpr int MaxPoints = 4;

    Contact();

    void SetBodies(RigidBody* bodyA, RigidBody* bodyB);

    RigidBody* GetBodyA();
    const RigidBody* GetBodyA() const;

    RigidBody* GetBodyB();
    const RigidBody* GetBodyB() const;

    void SetNormal(const Vec3& normal);
    const Vec3& GetNormal() const;

    const Vec3& GetTangent1() const;
    const Vec3& GetTangent2() const;

    void SetRestitution(float restitution);
    float GetRestitution() const;

    void SetFriction(float friction);
    float GetFriction() const;

    void AddPoint(
        const Vec3& pointOnA,
        const Vec3& pointOnB,
        float penetration
    );

    int GetPointCount() const;

    ContactPoint& GetPoint(int index);
    const ContactPoint& GetPoint(int index) const;

    void ClearPoints();

    void WarmStartFrom(const Contact& previous);

private:
    void BuildTangentBasis();

private:
    RigidBody* m_bodyA;
    RigidBody* m_bodyB;

    Vec3 m_normal;
    Vec3 m_tangent1;
    Vec3 m_tangent2;

    ContactPoint m_points[MaxPoints];
    int m_pointCount;

    float m_restitution;
    float m_friction;
};