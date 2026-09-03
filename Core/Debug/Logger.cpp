#include "Logger.h"

#include <iostream>
#include <fstream>
#include <mutex>

namespace {

    std::ofstream g_logFile;
    std::mutex g_logMutex;

    void Write(
        const char* level,
        const std::string& message
    ) {
        std::lock_guard<std::mutex> lock(g_logMutex);

        const std::string line =
            "[" + std::string(level) + "] " + message;

        std::cout << line << '\n';

        if (g_logFile.is_open()) {
            g_logFile << line << '\n';
            g_logFile.flush();
        }
    }

}

namespace Logger {

    void Initialize(const std::string& filePath) {
        std::lock_guard<std::mutex> lock(g_logMutex);

        g_logFile.open(
            filePath,
            std::ios::out | std::ios::trunc
        );
    }

    void Shutdown() {
        std::lock_guard<std::mutex> lock(g_logMutex);

        if (g_logFile.is_open())
            g_logFile.close();
    }

    void Info(const std::string& message) {
        Write("INFO", message);
    }

    void Debug(const std::string& message) {
        Write("DEBUG", message);
    }

    void Warning(const std::string& message) {
        Write("WARN", message);
    }

    void Error(const std::string& message) {
        Write("ERROR", message);
    }

}