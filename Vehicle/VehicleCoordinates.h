#pragma once

#include "../Core/Math/Vec3.h"

namespace VehicleCoordinates {
    inline Vec3 Forward() {
        return Vec3(0.0f, 0.0f, 1.0f);
    }

    inline Vec3 Right() {
        return Vec3(-1.0f, 0.0f, 0.0f);
    }

    inline Vec3 Left() {
        return Vec3(1.0f, 0.0f, 0.0f);
    }

    inline Vec3 Up() {
        return Vec3(0.0f, 1.0f, 0.0f);
    }

    inline Vec3 RightWheelPosition(float lateralDistance, float y, float z) {
        return Right() * lateralDistance + Vec3(0.0f, y, z);
    }

    inline Vec3 LeftWheelPosition(float lateralDistance, float y, float z) {
        return Left() * lateralDistance + Vec3(0.0f, y, z);
    }
}
