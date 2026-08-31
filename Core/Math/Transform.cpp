#include "Transform.h"

Transform::Transform()
    : position(0.0f, 0.0f, 0.0f),
    rotation(0.0f, 0.0f, 0.0f),
    scale(1.0f, 1.0f, 1.0f) {}

Mat4 Transform::GetMatrix() const {
    const Mat4 translation =
        Mat4::Translation(position);

    const Mat4 rotationX =
        Mat4::RotationX(rotation.x);

    const Mat4 rotationY =
        Mat4::RotationY(rotation.y);

    const Mat4 rotationZ =
        Mat4::RotationZ(rotation.z);

    const Mat4 scaling =
        Mat4::Scale(scale);

    return translation *
        rotationY *
        rotationX *
        rotationZ *
        scaling;
}