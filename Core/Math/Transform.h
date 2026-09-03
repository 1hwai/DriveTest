#pragma once

#include "Vec3.h"
#include "Mat4.h"
#include "Quaternion.h"

class Transform {
public:
    Transform();

    Vec3 position;
    Quaternion rotation;
    Vec3 scale;

    Mat4 GetMatrix() const;
};