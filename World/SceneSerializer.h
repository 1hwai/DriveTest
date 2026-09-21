#pragma once

#include <string>

class World;

class SceneSerializer {
public:
    static bool Save(const World& world, const std::string& path);
    static bool Load(World& world, const std::string& path);
};
