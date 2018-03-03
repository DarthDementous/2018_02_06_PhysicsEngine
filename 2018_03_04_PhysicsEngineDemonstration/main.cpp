#include "_2018_03_04_PhysicsEngineDemonstrationApp.h"

int main() {
	
	// allocation
	auto app = new _2018_03_04_PhysicsEngineDemonstrationApp();

	// initialise and loop
	app->run("AIE", 1280, 720, false);

	// deallocation
	delete app;

	return 0;
}