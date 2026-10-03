#pragma once

#include <filesystem>
#include <string>

#include "VehicleConfig.h"

enum class VehicleConfigReloadResult {
    Unchanged,
    Reloaded,
    Error
};

// Watches one vehicle's config file. Parsing/validation lives here;
// applying values to the live vehicle remains the caller's responsibility.
class VehicleConfigWatcher {
public:
    bool Initialize(const std::string& path, std::string& error);
    VehicleConfigReloadResult Poll(float deltaTime, VehicleConfig& config, std::string& error);
    const std::string& GetPath() const;

private:
    std::string m_path;
    std::filesystem::file_time_type m_lastWriteTime{};
    float m_checkTimer = 0.0f;
    bool m_initialized = false;
    static constexpr float CheckInterval = 0.5f;
};
