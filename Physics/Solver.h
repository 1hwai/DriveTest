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

    void SetRestitutionThreshold(float threshold);
    float GetRestitutionThreshold() const;

    void SolvePosition(Contact& contact);
    void SolveVelocity(Contact& contact);
    void SolveFriction(Contact& contact);

private:
    float m_penetrationSlop;
    float m_correctionPercent;
    int m_velocityIterations;
    float m_restitutionThreshold;
};