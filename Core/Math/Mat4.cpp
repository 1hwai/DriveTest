#include "Mat4.h"

#include <cmath>

Mat4::Mat4()
    : m{} {}

Mat4 Mat4::Identity() {
    Mat4 result;

    result.m[0][0] = 1.0f;
    result.m[1][1] = 1.0f;
    result.m[2][2] = 1.0f;
    result.m[3][3] = 1.0f;

    return result;
}

Mat4 Mat4::Translation(const Vec3& position) {
    Mat4 result = Identity();

    result.m[0][3] = position.x;
    result.m[1][3] = position.y;
    result.m[2][3] = position.z;

    return result;
}

Mat4 Mat4::Scale(const Vec3& scale) {
    Mat4 result = Identity();

    result.m[0][0] = scale.x;
    result.m[1][1] = scale.y;
    result.m[2][2] = scale.z;

    return result;
}

Mat4 Mat4::RotationX(float angle) {
    Mat4 result = Identity();

    const float c = std::cos(angle);
    const float s = std::sin(angle);

    result.m[1][1] = c;
    result.m[1][2] = -s;
    result.m[2][1] = s;
    result.m[2][2] = c;

    return result;
}

Mat4 Mat4::RotationY(float angle) {
    Mat4 result = Identity();

    const float c = std::cos(angle);
    const float s = std::sin(angle);

    result.m[0][0] = c;
    result.m[0][2] = s;
    result.m[2][0] = -s;
    result.m[2][2] = c;

    return result;
}

Mat4 Mat4::RotationZ(float angle) {
    Mat4 result = Identity();

    const float c = std::cos(angle);
    const float s = std::sin(angle);

    result.m[0][0] = c;
    result.m[0][1] = -s;
    result.m[1][0] = s;
    result.m[1][1] = c;

    return result;
}

Mat4 Mat4::Perspective(
    float fovY,
    float aspect,
    float nearPlane,
    float farPlane) {

    Mat4 result;

    const float tanHalfFov = std::tan(fovY * 0.5f);

    result.m[0][0] = 1.0f / (aspect * tanHalfFov);
    result.m[1][1] = 1.0f / tanHalfFov;

    result.m[2][2] =
        -(farPlane + nearPlane) /
        (farPlane - nearPlane);

    result.m[2][3] =
        -(2.0f * farPlane * nearPlane) /
        (farPlane - nearPlane);

    result.m[3][2] = -1.0f;

    return result;
}

Mat4 Mat4::operator*(const Mat4& other) const {
    Mat4 result;

    for (int row = 0; row < 4; ++row) {
        for (int column = 0; column < 4; ++column) {

            for (int k = 0; k < 4; ++k) {
                result.m[row][column] +=
                    m[row][k] * other.m[k][column];
            }
        }
    }

    return result;
}

Vec3 Mat4::TransformPoint(const Vec3& point) const {
    const float x =
        m[0][0] * point.x +
        m[0][1] * point.y +
        m[0][2] * point.z +
        m[0][3];

    const float y =
        m[1][0] * point.x +
        m[1][1] * point.y +
        m[1][2] * point.z +
        m[1][3];

    const float z =
        m[2][0] * point.x +
        m[2][1] * point.y +
        m[2][2] * point.z +
        m[2][3];

    const float w =
        m[3][0] * point.x +
        m[3][1] * point.y +
        m[3][2] * point.z +
        m[3][3];

    if (std::abs(w) > 0.000001f) {
        return Vec3(
            x / w,
            y / w,
            z / w
        );
    }

    return Vec3(x, y, z);
}

Vec3 Mat4::TransformVector(const Vec3& vector) const {
    return Vec3(
        m[0][0] * vector.x +
        m[0][1] * vector.y +
        m[0][2] * vector.z,

        m[1][0] * vector.x +
        m[1][1] * vector.y +
        m[1][2] * vector.z,

        m[2][0] * vector.x +
        m[2][1] * vector.y +
        m[2][2] * vector.z
    );
}