#include "Quaternion.h"

#include <cmath>

Quaternion::Quaternion()
    : w(1.0f),
    x(0.0f),
    y(0.0f),
    z(0.0f) {}

Quaternion::Quaternion(
    float w,
    float x,
    float y,
    float z
)
    : w(w),
    x(x),
    y(y),
    z(z) {}

Quaternion Quaternion::Identity() {
    return Quaternion(
        1.0f,
        0.0f,
        0.0f,
        0.0f
    );
}

Quaternion Quaternion::FromAxisAngle(
    const Vec3& axis,
    float angle
) {
    const Vec3 normalizedAxis =
        axis.Normalized();

    const float halfAngle =
        angle * 0.5f;

    const float sinHalf =
        std::sin(halfAngle);

    const float cosHalf =
        std::cos(halfAngle);

    return Quaternion(
        cosHalf,
        normalizedAxis.x * sinHalf,
        normalizedAxis.y * sinHalf,
        normalizedAxis.z * sinHalf
    );
}

float Quaternion::LengthSquared() const {
    return
        w * w +
        x * x +
        y * y +
        z * z;
}

float Quaternion::Length() const {
    return std::sqrt(LengthSquared());
}

Quaternion Quaternion::Normalized() const {
    const float length = Length();

    constexpr float epsilon = 0.000001f;

    if (length <= epsilon)
        return Identity();

    return Quaternion(
        w / length,
        x / length,
        y / length,
        z / length
    );
}

void Quaternion::Normalize() {
    const float length = Length();

    constexpr float epsilon = 0.000001f;

    if (length <= epsilon) {
        *this = Identity();
        return;
    }

    w /= length;
    x /= length;
    y /= length;
    z /= length;
}

Quaternion Quaternion::Conjugate() const {
    return Quaternion(
        w,
        -x,
        -y,
        -z
    );
}

Quaternion Quaternion::operator*(
    const Quaternion& other
    ) const {
    return Quaternion(
        w * other.w -
        x * other.x -
        y * other.y -
        z * other.z,

        w * other.x +
        x * other.w +
        y * other.z -
        z * other.y,

        w * other.y -
        x * other.z +
        y * other.w +
        z * other.x,

        w * other.z +
        x * other.y -
        y * other.x +
        z * other.w
    );
}

Quaternion& Quaternion::operator*=(
    const Quaternion& other
    ) {
    *this = *this * other;
    return *this;
}

Vec3 Quaternion::operator*(
    const Vec3& vector
    ) const {
    const Vec3 qVector(x, y, z);

    const Vec3 t =
        qVector.Cross(vector) * 2.0f;

    return vector +
        t * w +
        qVector.Cross(t);
}

Mat3 Quaternion::ToMat3() const {
    const Quaternion q = Normalized();

    const float xx = q.x * q.x;
    const float yy = q.y * q.y;
    const float zz = q.z * q.z;

    const float xy = q.x * q.y;
    const float xz = q.x * q.z;
    const float yz = q.y * q.z;

    const float wx = q.w * q.x;
    const float wy = q.w * q.y;
    const float wz = q.w * q.z;

    Mat3 result;

    result.m[0][0] = 1.0f - 2.0f * (yy + zz);
    result.m[0][1] = 2.0f * (xy - wz);
    result.m[0][2] = 2.0f * (xz + wy);

    result.m[1][0] = 2.0f * (xy + wz);
    result.m[1][1] = 1.0f - 2.0f * (xx + zz);
    result.m[1][2] = 2.0f * (yz - wx);

    result.m[2][0] = 2.0f * (xz - wy);
    result.m[2][1] = 2.0f * (yz + wx);
    result.m[2][2] = 1.0f - 2.0f * (xx + yy);

    return result;
}

Quaternion Quaternion::FromMat3(
    const Mat3& matrix
) {
    const float trace =
        matrix.m[0][0] +
        matrix.m[1][1] +
        matrix.m[2][2];

    Quaternion result;

    if (trace > 0.0f) {
        const float s =
            std::sqrt(trace + 1.0f) * 2.0f;

        result.w = 0.25f * s;
        result.x =
            (matrix.m[2][1] - matrix.m[1][2]) / s;
        result.y =
            (matrix.m[0][2] - matrix.m[2][0]) / s;
        result.z =
            (matrix.m[1][0] - matrix.m[0][1]) / s;
    }
    else if (
        matrix.m[0][0] >
        matrix.m[1][1] &&
        matrix.m[0][0] >
        matrix.m[2][2]
        ) {
        const float s =
            std::sqrt(
                1.0f +
                matrix.m[0][0] -
                matrix.m[1][1] -
                matrix.m[2][2]
            ) * 2.0f;

        result.w =
            (matrix.m[2][1] - matrix.m[1][2]) / s;

        result.x = 0.25f * s;

        result.y =
            (matrix.m[0][1] + matrix.m[1][0]) / s;

        result.z =
            (matrix.m[0][2] + matrix.m[2][0]) / s;
    }
    else if (
        matrix.m[1][1] >
        matrix.m[2][2]
        ) {
        const float s =
            std::sqrt(
                1.0f +
                matrix.m[1][1] -
                matrix.m[0][0] -
                matrix.m[2][2]
            ) * 2.0f;

        result.w =
            (matrix.m[0][2] - matrix.m[2][0]) / s;

        result.x =
            (matrix.m[0][1] + matrix.m[1][0]) / s;

        result.y = 0.25f * s;

        result.z =
            (matrix.m[1][2] + matrix.m[2][1]) / s;
    }
    else {
        const float s =
            std::sqrt(
                1.0f +
                matrix.m[2][2] -
                matrix.m[0][0] -
                matrix.m[1][1]
            ) * 2.0f;

        result.w =
            (matrix.m[1][0] - matrix.m[0][1]) / s;

        result.x =
            (matrix.m[0][2] + matrix.m[2][0]) / s;

        result.y =
            (matrix.m[1][2] + matrix.m[2][1]) / s;

        result.z = 0.25f * s;
    }

    return result.Normalized();
}