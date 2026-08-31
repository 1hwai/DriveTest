#pragma once

#include "../Core/Math/Vec3.h"
#include "../Core/Math/Mat4.h"

class Camera {
public:
    Camera();

    void SetPosition(const Vec3& position);
    void SetRotation(const Vec3& rotation);

    void SetAspectRatio(float aspectRatio);

    void Move(const Vec3& offset);
    void MoveLocal(const Vec3& offset);

    void Rotate(const Vec3& offset);

    const Vec3& GetPosition() const;
    const Vec3& GetRotation() const;

    Vec3 GetForward() const;
    Vec3 GetRight() const;
    Vec3 GetUp() const;

    Mat4 GetViewMatrix() const;
    Mat4 GetProjectionMatrix() const;

private:
    Vec3 m_position;
    Vec3 m_rotation;

    float m_fovY;
    float m_aspectRatio;
    float m_nearPlane;
    float m_farPlane;
};