#include "Vec3.h"

#include <cmath>

Vec3::Vec3()
    : x(0.0f),
    y(0.0f),
    z(0.0f) {}

Vec3::Vec3(float x, float y, float z)
    : x(x),
    y(y),
    z(z) {}

Vec3 Vec3::operator+(const Vec3& other) const {
    return Vec3(
        x + other.x,
        y + other.y,
        z + other.z
    );
}

Vec3 Vec3::operator-(const Vec3& other) const {
    return Vec3(
        x - other.x,
        y - other.y,
        z - other.z
    );
}

Vec3 Vec3::operator*(float scalar) const {
    return Vec3(
        x * scalar,
        y * scalar,
        z * scalar
    );
}

Vec3 Vec3::operator/(float scalar) const {
    return Vec3(
        x / scalar,
        y / scalar,
        z / scalar
    );
}

Vec3& Vec3::operator+=(const Vec3& other) {
    x += other.x;
    y += other.y;
    z += other.z;

    return *this;
}

Vec3& Vec3::operator-=(const Vec3& other) {
    x -= other.x;
    y -= other.y;
    z -= other.z;

    return *this;
}

Vec3& Vec3::operator*=(float scalar) {
    x *= scalar;
    y *= scalar;
    z *= scalar;

    return *this;
}

Vec3& Vec3::operator/=(float scalar) {
    x /= scalar;
    y /= scalar;
    z /= scalar;

    return *this;
}

float Vec3::LengthSquared() const {
    return x * x + y * y + z * z;
}

float Vec3::Length() const {
    return std::sqrt(LengthSquared());
}

Vec3 Vec3::Normalized() const {
    const float length = Length();

    if (length <= 0.000001f)
        return Vec3();

    return *this / length;
}

float Vec3::Dot(const Vec3& other) const {
    return
        x * other.x +
        y * other.y +
        z * other.z;
}

Vec3 Vec3::Cross(const Vec3& other) const {
    return Vec3(
        y * other.z - z * other.y,
        z * other.x - x * other.z,
        x * other.y - y * other.x
    );
}