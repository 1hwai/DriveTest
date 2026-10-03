#include "VehicleConfigWatcher.h"

#include <system_error>

bool VehicleConfigWatcher::Initialize(const std::string& path, std::string& error) {
    std::error_code timestampError;
    const auto writeTime = std::filesystem::last_write_time(path, timestampError);
    if (timestampError) {
        error = "Unable to watch vehicle config '" + path + "': " + timestampError.message();
        m_initialized = false;
        return false;
    }

    m_path = path;
    m_lastWriteTime = writeTime;
    m_checkTimer = 0.0f;
    m_initialized = true;
    return true;
}

VehicleConfigReloadResult VehicleConfigWatcher::Poll(
    float deltaTime,
    VehicleConfig& config,
    std::string& error
) {
    if (!m_initialized || m_path.empty()) {
        error = "Vehicle config watcher is not initialized";
        return VehicleConfigReloadResult::Error;
    }

    m_checkTimer += deltaTime;
    if (m_checkTimer < CheckInterval)
        return VehicleConfigReloadResult::Unchanged;
    m_checkTimer = 0.0f;

    std::error_code timestampError;
    const auto writeTime = std::filesystem::last_write_time(m_path, timestampError);
    if (timestampError) {
        error = "Unable to check vehicle config '" + m_path + "': " + timestampError.message();
        return VehicleConfigReloadResult::Error;
    }

    if (writeTime == m_lastWriteTime)
        return VehicleConfigReloadResult::Unchanged;

    VehicleConfig candidate;
    if (!candidate.Load(m_path, error)) {
        m_lastWriteTime = writeTime;
        return VehicleConfigReloadResult::Error;
    }

    config = candidate;
    m_lastWriteTime = writeTime;
    return VehicleConfigReloadResult::Reloaded;
}

const std::string& VehicleConfigWatcher::GetPath() const {
    return m_path;
}
