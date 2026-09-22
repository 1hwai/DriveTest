#include "Shader.h"

#include <glad/gl.h>

#include <fstream>
#include <sstream>

namespace {
    bool ReadFile(const std::string& path, std::string& output) {
        std::ifstream file(path);

        if (!file.is_open())
            return false;

        std::stringstream stream;
        stream << file.rdbuf();

        output = stream.str();

        return true;
    }

    bool CheckShader(unsigned int shader) {
        int success = 0;
        glGetShaderiv(shader, GL_COMPILE_STATUS, &success);

        if (success)
            return true;

        char infoLog[1024];
        glGetShaderInfoLog(shader, sizeof(infoLog), nullptr, infoLog);

        return false;
    }

    bool CheckProgram(unsigned int program) {
        int success = 0;
        glGetProgramiv(program, GL_LINK_STATUS, &success);

        if (success)
            return true;

        char infoLog[1024];
        glGetProgramInfoLog(program, sizeof(infoLog), nullptr, infoLog);

        return false;
    }
}

Shader::Shader()
    : m_program(0) {}

Shader::~Shader() {
    Destroy();
}

bool Shader::Load(
    const std::string& vertexPath,
    const std::string& fragmentPath) {

    std::string vertexSource;
    std::string fragmentSource;

    if (!ReadFile(vertexPath, vertexSource))
        return false;

    if (!ReadFile(fragmentPath, fragmentSource))
        return false;

    const char* vertexCode = vertexSource.c_str();
    const char* fragmentCode = fragmentSource.c_str();

    unsigned int vertexShader = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vertexShader, 1, &vertexCode, nullptr);
    glCompileShader(vertexShader);

    if (!CheckShader(vertexShader)) {
        glDeleteShader(vertexShader);
        return false;
    }

    unsigned int fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(fragmentShader, 1, &fragmentCode, nullptr);
    glCompileShader(fragmentShader);

    if (!CheckShader(fragmentShader)) {
        glDeleteShader(vertexShader);
        glDeleteShader(fragmentShader);
        return false;
    }

    m_program = glCreateProgram();

    glAttachShader(m_program, vertexShader);
    glAttachShader(m_program, fragmentShader);
    glLinkProgram(m_program);

    glDeleteShader(vertexShader);
    glDeleteShader(fragmentShader);

    if (!CheckProgram(m_program)) {
        glDeleteProgram(m_program);
        m_program = 0;
        return false;
    }

    m_uniformLocations.clear();

    return true;
}

void Shader::Use() const {
    if (m_program != 0)
        glUseProgram(m_program);
}

void Shader::Destroy() {
    if (m_program != 0) {
        glDeleteProgram(m_program);
        m_program = 0;
    }

    m_uniformLocations.clear();
}

int Shader::GetUniformLocation(const char* name) {
    const auto found = m_uniformLocations.find(name);

    if (found != m_uniformLocations.end())
        return found->second;

    const int location = glGetUniformLocation(m_program, name);
    m_uniformLocations.emplace(name, location);

    return location;
}

void Shader::SetMat4(const char* name, const Mat4& matrix) {
    const int location = GetUniformLocation(name);

    if (location == -1)
        return;

    glUniformMatrix4fv(
        location,
        1,
        GL_TRUE,
        &matrix.m[0][0]
    );
}

void Shader::SetMat3(const char* name, const Mat3& matrix) {
    const int location = GetUniformLocation(name);

    if (location == -1)
        return;

    glUniformMatrix3fv(
        location,
        1,
        GL_TRUE,
        &matrix.m[0][0]
    );
}

void Shader::SetVec3(const char* name, const Vec3& vector) {
    const int location = GetUniformLocation(name);

    if (location == -1)
        return;

    glUniform3f(
        location,
        vector.x,
        vector.y,
        vector.z
    );
}