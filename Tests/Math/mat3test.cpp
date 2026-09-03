#include <iostream>
#include "Core/Math/Mat3.h"
#include "Core/Math/Vec3.h"

int main() {
    Mat3 identity = Mat3::Identity();

    Vec3 value(1.0f, 2.0f, 3.0f);
    Vec3 result = identity * value;

    std::cout
        << result.x << ", "
        << result.y << ", "
        << result.z << std::endl;

    return 0;
}