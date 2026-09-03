#pragma once

#include "Vec3.h"

class Mat3 {
public:
    float m[3][3];

    Mat3();

    static Mat3 Identity();

    Vec3 operator*(const Vec3& vector) const;
    Mat3 operator*(const Mat3& other) const;
    Mat3 operator*(float scalar) const;

    Mat3 Transposed() const;

    float Determinant() const;
    Mat3 Inversed() const;
};