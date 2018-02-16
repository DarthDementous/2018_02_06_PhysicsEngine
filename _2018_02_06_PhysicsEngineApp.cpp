#include "_2018_02_06_PhysicsEngineApp.h"
#include "Gizmos.h"
#include "Input.h"
#include <glm/glm.hpp>
#include <glm/ext.hpp>
#include "Physics\Sphere.h"
#include "Physics\Scene.h"
#include "Camera\Camera.h"
#include <algorithm>
#include <imgui.h>

using glm::vec3;
using glm::vec4;
using glm::mat4;
using aie::Gizmos;
using namespace Physebs;

_2018_02_06_PhysicsEngineApp::_2018_02_06_PhysicsEngineApp() {

}

_2018_02_06_PhysicsEngineApp::~_2018_02_06_PhysicsEngineApp() {

}

bool _2018_02_06_PhysicsEngineApp::startup() {
	
	setBackgroundColour(0.25f, 0.25f, 0.25f);

	// initialise gizmo primitive counts
	Gizmos::create(10000, 10000, 10000, 10000);

	//// create simple camera transforms
	//m_viewMatrix = glm::lookAt(vec3(10), vec3(0), vec3(0, 1, 0));
	//m_projectionMatrix = glm::perspective(glm::pi<float>() * 0.25f, 16.0f / 9.0f, 0.1f, 1000.0f);

	// Custom camera calibration
	m_camera = new Camera();
	m_camera->SetProjection(glm::radians(45.0f), (float)getWindowWidth() / (float)getWindowHeight(), 0.1f, 1000.0f);
	m_camera->SetPosition(glm::vec3(10, 10, 10));
	m_camera->Lookat(glm::vec3(0, 0, 0));

	// Make a scene (ha-ha)
	m_scene = new Scene();
	m_scene->SetGlobalForce(glm::vec3(0.f, 0, 0));
	
#pragma region Manual Object Creation
	//static const float massStep = 4.f;

	//// Default weight (RED)
	///*m_scene->AddObject(new Sphere(DEFAULT_MASS, glm::vec2(16.f, 16.f),
	//glm::vec3(0, DEFAULT_MASS, 0), DEFAULT_MASS, 4.f, true, glm::vec4(1.f, 0.f, 0.f, 1.f)));*/

	//// Light weight (GREEN)
	//float mass = std::max(DEFAULT_MASS - massStep, 1.f);		// Ensure mass does not end up negative or else object will accelerate in the wrong direction
	//m_scene->AddObject(new Sphere(mass, glm::vec2(16.f, 16.f),
	//	glm::vec3(massStep, mass - massStep, 0), mass, 4.f, true, glm::vec4(0.f, 1.f, 0.f, 1.f)));

	//// Heavy weight (BLUE)
	//Sphere* bigboi = new Sphere(DEFAULT_MASS + massStep, glm::vec2(16.f, 16.f),
	//	glm::vec3(-massStep, DEFAULT_MASS + massStep, 0), DEFAULT_MASS + massStep, 4.f, true, glm::vec4(0.f, 0.f, 1.f, 1.f));
	//bigboi->ApplyImpulseForce(glm::vec3(10.f, 0.f, 0.f));

	//m_scene->AddObject(bigboi);
#pragma endregion

	
	return true;
}

void _2018_02_06_PhysicsEngineApp::shutdown() {

	delete m_camera;
	delete m_scene;
	
	Gizmos::destroy();
}

void _2018_02_06_PhysicsEngineApp::update(float deltaTime) {

	// wipe the gizmos clean for this frame
	Gizmos::clear();

	// quit if we press escape
	aie::Input* input = aie::Input::getInstance();

#pragma region IMGUI
	//ImGui::Begin("Gravity Debug");

	//glm::vec3 currentObjVel = m_scene->GetObjects()[1]->GetVel();
	//ImGui::Text("Heavy Object #1 (Blue) Velocity: %.6f, %.6f, %.6f", currentObjVel.x, currentObjVel.y, currentObjVel.z);

	/// Object Creator GUI
#if 0
	ImGui::ShowTestWindow();
#else 
	ImGui::Begin("Object Creator");

	// Make variables static so they're only defined once so their values at their addresses can be changed after by user input
	static float	globalForce[3] = { 0.f, 0.f, 0.f };		// TODO: Put this into seperate section from Object Creator in ImGui

	static float	pos[3]		= { 0.f, 0.f, 0.f };
	static float	force[3]	= { 0.f, 0.f, 0.f };
	static float	dim[2]		= { DEFAULT_SPHERE.x, DEFAULT_SPHERE.y };
	static float	radius		= DEFAULT_MASS;
	static float	mass		= DEFAULT_MASS;
	static float	friction	= DEFAULT_FRICTION;
	static float	color[4]	= { 0.f, 0.f, 0.f, 1.f };
	static bool		b_dynamic	= true;
	static bool		b_impulse	= true;

	ImGui::InputFloat3("Scene Global Force", globalForce, 2);

	ImGui::InputFloat3("Position", pos, 2);
	ImGui::InputFloat3("Starting Force", force, 2);
	ImGui::InputFloat2("Dimensions", dim, 2);
	ImGui::InputFloat("Radius", &radius, 1.f, 0.f, 2);
	ImGui::InputFloat("Mass", &mass, 1.f, 0.f, 2);
	ImGui::InputFloat("Friction", &friction, 1.f, 0.f, 2);
	ImGui::ColorEdit4("Color", color);
	ImGui::Checkbox("Is Dynamic", &b_dynamic);
	ImGui::Checkbox("Velocity is impulse", &b_impulse);

	// Object spawn button has been clicked
	if (ImGui::SmallButton("Spawn Sphere")) {
		// Grab data from static input variables and create object from it
		glm::vec3 currentPos	= glm::vec3(pos[0], pos[1], pos[2]);
		glm::vec3 currentForce	= glm::vec3(force[0], force[1], force[2]);
		glm::vec2 currentDim	= glm::vec2(dim[0], dim[1]);
		glm::vec4 currentColor	= glm::vec4(color[0], color[1], color[2], color[3]);

		Sphere* newObj = new Sphere(radius, currentDim, currentPos, mass, friction, b_dynamic, currentColor);

		// Object velocity is not impulse, apply over time
		if (!b_impulse) {
			newObj->ApplyForce(currentForce);
		}
		// Object velocity is impulse, apply instantly
		else {
			newObj->ApplyImpulseForce(currentForce);
		}

		// Add created object to scene
		m_scene->AddObject(newObj);
	}

	glm::vec3 currentGlobalForce = glm::vec3(globalForce[0], globalForce[1], globalForce[2]);
	
	ImGui::End();
#endif
#pragma endregion


	/// Object control
	//glm::vec3 movementForce = glm::vec3();
	//static const float maxSpeed = 1000.f;

	//if (input->isKeyDown(aie::INPUT_KEY_UP)) {
	//	movementForce.z += maxSpeed * deltaTime;
	//}
	//if (input->isKeyDown(aie::INPUT_KEY_DOWN)) {
	//	movementForce.z -= maxSpeed * deltaTime;
	//}
	//if (input->isKeyDown(aie::INPUT_KEY_LEFT)) {
	//	movementForce.x += maxSpeed * deltaTime;
	//}
	//if (input->isKeyDown(aie::INPUT_KEY_RIGHT)) {
	//	movementForce.x -= maxSpeed * deltaTime;
	//}

	//m_scene->SetGlobalForce(movementForce);

	//m_object->ApplyForce(movementForce);

	//m_object->Update(deltaTime);

	/// Scene
	m_scene->SetGlobalForce(currentGlobalForce);
	m_scene->ApplyGlobalForce();
	m_scene->FixedUpdate(deltaTime);

	/// Camera
	//glm::vec3 currentPos = m_object->GetPos();

	//m_camera->SetPosition(glm::vec3(currentPos.x, currentPos.y, currentPos.z + 10.f));
	//m_camera->Lookat(m_object->GetPos());

	m_camera->Update(deltaTime);

#pragma region 3D Template Code
	// draw a simple grid with gizmos
	vec4 white(1);
	vec4 black(0, 0, 0, 1);
	for (int i = 0; i < 21; ++i) {
		Gizmos::addLine(vec3(-10 + i, 0, 10),
			vec3(-10 + i, 0, -10),
			i == 10 ? white : black);
		Gizmos::addLine(vec3(10, 0, -10 + i),
			vec3(-10, 0, -10 + i),
			i == 10 ? white : black);
	}

	// add a transform so that we can see the axis
	Gizmos::addTransform(mat4(1));

#pragma endregion

	if (input->isKeyDown(aie::INPUT_KEY_ESCAPE))
		quit();
}

void _2018_02_06_PhysicsEngineApp::draw() {

	// wipe the screen to the background colour
	clearScreen();

	/// Scene
	m_scene->Draw();
	
	Gizmos::draw(m_camera->GetProjectionView());

#pragma region 3D Template Code
	//// update perspective based on screen size
	//m_projectionMatrix = glm::perspective(glm::pi<float>() * 0.25f, getWindowWidth() / (float)getWindowHeight(), 0.1f, 1000.0f);

	//Gizmos::draw(m_projectionMatrix * m_viewMatrix);
#pragma endregion
}