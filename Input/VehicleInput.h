#pragma once

struct VehicleInput {
    float throttle = 0.0f;
    float brake = 0.0f;
    float steering = 0.0f;
    float clutch = 0.0f;
    bool shiftUp = false;
    bool shiftDown = false;
};
