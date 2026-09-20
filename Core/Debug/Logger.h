#pragma once

#include <string>

namespace Logger {

    void Initialize(const std::string& filePath);
    void Initialize();
    void Shutdown();

    void Info(const std::string& message);
    void Debug(const std::string& message);
    void Warning(const std::string& message);
    void Error(const std::string& message);

}