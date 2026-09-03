#pragma once

#include "Vec3.h"
#include "Mat3.h"

class Mat4 {
public:
    float m[4][4];

    Mat4();

    static Mat4 Identity();
    static Mat4 Translation(const Vec3& position);
    static Mat4 Scale(const Vec3& scale);
    static Mat4 FromMat3(const Mat3& matrix);

    static Mat4 RotationX(float angle);
    static Mat4 RotationY(float angle);
    static Mat4 RotationZ(float angle);

    static Mat4 Perspective(
        float fovY,
        float aspect,
        float nearPlane,
        float farPlane
    );

    Mat4 operator*(const Mat4& other) const;

    Vec3 TransformPoint(const Vec3& point) const;
    Vec3 TransformVector(const Vec3& vector) const;
};