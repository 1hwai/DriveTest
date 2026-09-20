#include <iostream>
#include "Core/Application.h"
#include "Core/Debug/Logger.h"

int main() {
	Logger::Initialize();
	Application app;
	if (!app.Initialize()) {
		std::cerr << "Failed to initialize application." << std::endl;
		Logger::Shutdown();
		return -1;
	}

	const int result = app.Run();
	Logger::Shutdown();

	return result;
}