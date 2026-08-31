#pragma once

#include "Vec3.h"
#include "Mat4.h"

class Transform {
public:
    Transform();

    Vec3 position;
    Vec3 rotation;
    Vec3 scale;

    Mat4 GetMatrix() const;
};