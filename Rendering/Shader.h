#pragma once

#include <string>
#include <unordered_map>

#include "../Core/Math/Mat3.h"
#include "../Core/Math/Mat4.h"
#include "../Core/Math/Vec3.h"

class Shader {
public:
    Shader();
    ~Shader();

    bool Load(const std::string& vertexPath, const std::string& fragmentPath);
    void Use() const;
    void Destroy();

    void SetMat4(const char* name, const Mat4& matrix);
    void SetMat3(const char* name, const Mat3& matrix);
    void SetVec3(const char* name, const Vec3& vector);

private:
    int GetUniformLocation(const char* name);

    unsigned int m_program;
    std::unordered_map<std::string, int> m_uniformLocations;
};