#include "Camera.h"

#include <cmath>

namespace {
    constexpr float PI = 3.14159265359f;
}

Camera::Camera()
    : m_position(0.0f, 0.0f, 3.0f),
    m_rotation(0.0f, 0.0f, 0.0f),
    m_fovY(60.0f * PI / 180.0f),
    m_aspectRatio(16.0f / 9.0f),
    m_nearPlane(0.1f),
    m_farPlane(100.0f) {}

void Camera::SetPosition(const Vec3& position) {
    m_position = position;
}

void Camera::SetRotation(const Vec3& rotation) {
    m_rotation = rotation;
}

void Camera::SetAspectRatio(float aspectRatio) {
    if (aspectRatio <= 0.0f)
        return;

    m_aspectRatio = aspectRatio;
}

void Camera::Move(const Vec3& offset) {
    m_position += offset;
}

void Camera::Rotate(const Vec3& offset) {
    m_rotation += offset;
}

const Vec3& Camera::GetPosition() const {
    return m_position;
}

const Vec3& Camera::GetRotation() const {
    return m_rotation;
}

Vec3 Camera::GetForward() const {
    const Mat4 rotation =
        Mat4::RotationY(m_rotation.y) *
        Mat4::RotationX(m_rotation.x) *
        Mat4::RotationZ(m_rotation.z);

    return rotation.TransformVector(
        Vec3(0.0f, 0.0f, -1.0f)
    );
}

Vec3 Camera::GetRight() const {
    const Mat4 rotation =
        Mat4::RotationY(m_rotation.y) *
        Mat4::RotationX(m_rotation.x) *
        Mat4::RotationZ(m_rotation.z);

    return rotation.TransformVector(
        Vec3(1.0f, 0.0f, 0.0f)
    );
}

Vec3 Camera::GetUp() const {
    const Mat4 rotation =
        Mat4::RotationY(m_rotation.y) *
        Mat4::RotationX(m_rotation.x) *
        Mat4::RotationZ(m_rotation.z);

    return rotation.TransformVector(
        Vec3(0.0f, 1.0f, 0.0f)
    );
}

void Camera::MoveLocal(const Vec3& offset) {
    m_position +=
        GetRight() * offset.x +
        GetUp() * offset.y +
        GetForward() * offset.z;
}

Mat4 Camera::GetViewMatrix() const {
    const Mat4 rotationX =
        Mat4::RotationX(-m_rotation.x);

    const Mat4 rotationY =
        Mat4::RotationY(-m_rotation.y);

    const Mat4 rotationZ =
        Mat4::RotationZ(-m_rotation.z);

    const Mat4 translation =
        Mat4::Translation(
            Vec3(
                -m_position.x,
                -m_position.y,
                -m_position.z
            )
        );

    return rotationZ *
        rotationX *
        rotationY *
        translation;
}

Mat4 Camera::GetProjectionMatrix() const {
    return Mat4::Perspective(
        m_fovY,
        m_aspectRatio,
        m_nearPlane,
        m_farPlane
    );
}