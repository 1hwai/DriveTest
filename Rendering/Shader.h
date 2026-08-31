#pragma once

#include <string>
#include "../Core/Math/Mat4.h"

class Shader {
public:
    Shader();
    ~Shader();

    bool Load(const std::string& vertexPath, const std::string& fragmentPath);
    void Use() const;
    void Destroy();

    void SetMat4(const char* name, const Mat4& matrix);

private:
    unsigned int m_program;
};