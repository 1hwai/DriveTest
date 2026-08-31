#include <iostream>
#include "Core/Application.h"

int main() {
	Application app;
	if (!app.Initialize()) {
		std::cerr << "Failed to initialize application." << std::endl;
		return -1;
	}

	return app.Run();
}