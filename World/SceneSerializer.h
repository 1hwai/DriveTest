#pragma once

#include <string>

class Scene;

class SceneSerializer {
public:
    static bool Save(const Scene& world, const std::string& path);
    static bool Load(Scene& world, const std::string& path);
};
