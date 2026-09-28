#include "Differential.h"

#include <algorithm>

Differential::Differential() = default;

void Differential::DistributeTorque(
    float inputTorque,
    float& leftTorque,
    float& rightTorque
) const {
    leftTorque =
        inputTorque * 0.5f;

    rightTorque =
        inputTorque * 0.5f;
}

float Differential::GetInputLoadTorque(
    float leftReactionTorque,
    float rightReactionTorque
) const {
    return std::max(
        0.0f,
        -(leftReactionTorque + rightReactionTorque)
    );
}
