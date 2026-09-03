#pragma once

#include "Vec3.h"
#include "Mat3.h"

class Quaternion {
public:
    float w;
    float x;
    float y;
    float z;

    Quaternion();
    Quaternion(float w, float x, float y, float z);

    static Quaternion Identity();
    static Quaternion FromAxisAngle(
        const Vec3& axis,
        float angle
    );

    float LengthSquared() const;
    float Length() const;

    Quaternion Normalized() const;
    void Normalize();

    Quaternion Conjugate() const;

    Quaternion operator*(const Quaternion& other) const;
    Vec3 operator*(const Vec3& vector) const;

    Quaternion& operator*=(const Quaternion& other);

    Mat3 ToMat3() const;

    static Quaternion FromMat3(const Mat3& matrix);
};