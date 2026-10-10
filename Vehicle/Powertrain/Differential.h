#pragma once

class Differential {
public:
    Differential();

    void DistributeTorque(
        float inputTorque,
        float& leftTorque,
        float& rightTorque
    ) const;

    float GetInputLoadTorque(
        float leftReactionTorque,
        float rightReactionTorque
    ) const;
};
