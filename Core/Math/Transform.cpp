#include "Transform.h"

Transform::Transform()
    : position(0.0f, 0.0f, 0.0f),
    rotation(Quaternion::Identity()),
    scale(1.0f, 1.0f, 1.0f) {}

Mat4 Transform::GetMatrix() const {
    const Mat4 translation =
        Mat4::Translation(position);

    const Mat4 rotationMatrix =
        Mat4::FromMat3(rotation.ToMat3());

    const Mat4 scaling =
        Mat4::Scale(scale);

    return translation *
        rotationMatrix *
        scaling;
}