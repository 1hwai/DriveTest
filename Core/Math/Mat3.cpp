#include "Mat3.h"

#include <cmath>

Mat3::Mat3()
    : m{} {}

Mat3 Mat3::Identity() {
    Mat3 result;

    result.m[0][0] = 1.0f;
    result.m[1][1] = 1.0f;
    result.m[2][2] = 1.0f;

    return result;
}

Vec3 Mat3::operator*(const Vec3& vector) const {
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

Mat3 Mat3::operator*(const Mat3& other) const {
    Mat3 result;

    for (int row = 0; row < 3; ++row) {
        for (int column = 0; column < 3; ++column) {
            for (int k = 0; k < 3; ++k) {
                result.m[row][column] +=
                    m[row][k] * other.m[k][column];
            }
        }
    }

    return result;
}

Mat3 Mat3::operator*(float scalar) const {
    Mat3 result;

    for (int row = 0; row < 3; ++row) {
        for (int column = 0; column < 3; ++column) {
            result.m[row][column] =
                m[row][column] * scalar;
        }
    }

    return result;
}

Mat3 Mat3::Transposed() const {
    Mat3 result;

    for (int row = 0; row < 3; ++row) {
        for (int column = 0; column < 3; ++column) {
            result.m[row][column] =
                m[column][row];
        }
    }

    return result;
}

float Mat3::Determinant() const {
    return
        m[0][0] * (
            m[1][1] * m[2][2] -
            m[1][2] * m[2][1]
            )
        - m[0][1] * (
            m[1][0] * m[2][2] -
            m[1][2] * m[2][0]
            )
        + m[0][2] * (
            m[1][0] * m[2][1] -
            m[1][1] * m[2][0]
            );
}

Mat3 Mat3::Inversed() const {
    const float determinant = Determinant();

    constexpr float epsilon = 0.000001f;

    if (std::abs(determinant) <= epsilon)
        return Mat3();

    Mat3 result;

    result.m[0][0] =
        m[1][1] * m[2][2] -
        m[1][2] * m[2][1];

    result.m[0][1] =
        -(m[0][1] * m[2][2] -
            m[0][2] * m[2][1]);

    result.m[0][2] =
        m[0][1] * m[1][2] -
        m[0][2] * m[1][1];

    result.m[1][0] =
        -(m[1][0] * m[2][2] -
            m[1][2] * m[2][0]);

    result.m[1][1] =
        m[0][0] * m[2][2] -
        m[0][2] * m[2][0];

    result.m[1][2] =
        -(m[0][0] * m[1][2] -
            m[0][2] * m[1][0]);

    result.m[2][0] =
        m[1][0] * m[2][1] -
        m[1][1] * m[2][0];

    result.m[2][1] =
        -(m[0][0] * m[2][1] -
            m[0][1] * m[2][0]);

    result.m[2][2] =
        m[0][0] * m[1][1] -
        m[0][1] * m[1][0];

    return result * (1.0f / determinant);
}