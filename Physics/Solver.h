#pragma once

#include "Contact.h"

class Solver {
public:
    Solver();
    ~Solver() = default;

    void SetPenetrationSlop(float slop);
    float GetPenetrationSlop() const;

    void SetCorrectionPercent(float percent);
    float GetCorrectionPercent() const;

    void SetVelocityIterations(int iterations);
    int GetVelocityIterations() const;

    void SetPositionIterations(int iterations);
    int GetPositionIterations() const;

    void SetRestitutionThreshold(float threshold);
    float GetRestitutionThreshold() const;

    void WarmStart(Contact& contact);

    // 워밍스타트 직전, 프레임당 한 번만 호출: 이 순간의(=아직 이번 프레임
    // 접촉 임펄스가 하나도 안 걸린) 접근 속도로 반발 목표 속도를 계산해
    // 각 점에 고정시켜 둔다.
    void PrepareVelocityBias(Contact& contact);

    void SolvePosition(Contact& contact);
    void SolveVelocity(Contact& contact);
    void SolveFriction(Contact& contact);

private:
    float m_penetrationSlop;
    float m_correctionPercent;
    int m_velocityIterations;
    int m_positionIterations;
    float m_restitutionThreshold;
};