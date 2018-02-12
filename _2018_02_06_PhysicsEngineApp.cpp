#include "_2018_02_06_PhysicsEngineApp.h"
#include "Gizmos.h"
#include "Input.h"
#include <glm/glm.hpp>
#include <glm/ext.hpp>
#include "Physics\Rigidbody.h"
#include "Physics\Scene.h"
#include "Camera\Camera.h"

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
	
	// Create and add object to scene
	m_object = new Rigidbody(glm::vec3(), 2.f);
	m_scene->AddObject(m_object);

	return true;
}

void _2018_02_06_PhysicsEngineApp::shutdown() {

	delete m_camera;

	Gizmos::destroy();
}

void _2018_02_06_PhysicsEngineApp::update(float deltaTime) {

	// wipe the gizmos clean for this frame
	Gizmos::clear();

	glm::vec3 currentPos = m_object->GetPos();

	//m_camera->SetPosition(glm::vec3(currentPos.x, currentPos.y + 100.f, currentPos.z - 100.f));
	m_camera->Lookat(m_object->GetPos());

	m_camera->Update(deltaTime);

	aie::Gizmos::addSphere(m_object->GetPos(), m_object->GetMass(), 8, 8, glm::vec4(1.f, 0.f, 0.f, 1.f));

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
	
	// quit if we press escape
	aie::Input* input = aie::Input::getInstance();

	m_object->Update(deltaTime);

	// Object control
	glm::vec3 movementForce = glm::vec3();
	static const float maxSpeed = 1000.f;

	if (input->isKeyDown(aie::INPUT_KEY_UP)) {
		movementForce.z += maxSpeed * deltaTime;
	}
	if (input->isKeyDown(aie::INPUT_KEY_DOWN)) {
		movementForce.z -= maxSpeed * deltaTime;
	}
	if (input->isKeyDown(aie::INPUT_KEY_LEFT)) {
		movementForce.x += maxSpeed * deltaTime;
	}
	if (input->isKeyDown(aie::INPUT_KEY_RIGHT)) {
		movementForce.x -= maxSpeed * deltaTime;
	}

	m_object->ApplyForce(movementForce);

	if (input->isKeyDown(aie::INPUT_KEY_ESCAPE))
		quit();
}

void _2018_02_06_PhysicsEngineApp::draw() {

	// wipe the screen to the background colour
	clearScreen();

	Gizmos::draw(m_camera->GetProjectionView());

#pragma region 3D Template Code
	//// update perspective based on screen size
	//m_projectionMatrix = glm::perspective(glm::pi<float>() * 0.25f, getWindowWidth() / (float)getWindowHeight(), 0.1f, 1000.0f);

	//Gizmos::draw(m_projectionMatrix * m_viewMatrix);
#pragma endregion
}