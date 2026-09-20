#include "Logger.h"

#include <iostream>
#include <fstream>
#include <mutex>
#include <algorithm>
#include <filesystem>
#include <string>

namespace {

    std::ofstream g_logFile;
    std::mutex g_logMutex;

    void RotateLatestLog(const std::filesystem::path& logPath) {
        if (!std::filesystem::exists(logPath))
            return;

        const std::filesystem::path directory =
            logPath.parent_path().empty()
                ? std::filesystem::current_path()
                : logPath.parent_path();

        int nextNumber = 1;

        for (const auto& entry : std::filesystem::directory_iterator(directory)) {
            if (!entry.is_regular_file())
                continue;

            const std::string fileName = entry.path().filename().string();

            const std::string prefix = "test_";
            const std::string suffix = ".log";

            if (fileName.size() <= prefix.size() + suffix.size())
                continue;

            if (fileName.rfind(prefix, 0) != 0)
                continue;

            if (fileName.substr(fileName.size() - suffix.size()) != suffix)
                continue;

            const std::string numberText =
                fileName.substr(
                    prefix.size(),
                    fileName.size() - prefix.size() - suffix.size()
                );

            try {
                const int number = std::stoi(numberText);
                nextNumber = std::max(nextNumber, number + 1);
            }
            catch (...) {
                continue;
            }
        }

        const std::filesystem::path archivedPath =
            directory / ("test_" + std::to_string(nextNumber) + ".log");

        std::filesystem::rename(logPath, archivedPath);
    }

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

    void Initialize() {
        Initialize("Logs/latest.log");
    }

    void Initialize(const std::string& filePath) {
        std::lock_guard<std::mutex> lock(g_logMutex);

        if (g_logFile.is_open())
            g_logFile.close();

        const std::filesystem::path logPath(filePath);
        const std::filesystem::path directory = logPath.parent_path();

        if (!directory.empty())
            std::filesystem::create_directories(directory);

        RotateLatestLog(logPath);

        g_logFile.open(
            logPath,
            std::ios::out | std::ios::trunc
        );

        if (!g_logFile.is_open()) {
            std::cerr << "Failed to open log file: " << logPath << std::endl;
        }
    }

    void Shutdown() {
        std::lock_guard<std::mutex> lock(g_logMutex);

        if (g_logFile.is_open()) {
            g_logFile.flush();
            g_logFile.close();
        }
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