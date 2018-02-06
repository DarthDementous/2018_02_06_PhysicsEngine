#include "_2018_02_06_PhysicsEngineApp.h"

int main() {
	
	// allocation
	auto app = new _2018_02_06_PhysicsEngineApp();

	// initialise and loop
	app->run("AIE", 1280, 720, false);

	// deallocation
	delete app;

	return 0;
}