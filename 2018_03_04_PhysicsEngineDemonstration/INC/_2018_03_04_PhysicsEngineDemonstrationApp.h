#pragma once

#include "Application.h"
#include <glm/mat4x4.hpp>

namespace Physebs {
	class Rigidbody;
	class Scene;
}
class Camera;

class _2018_03_04_PhysicsEngineDemonstrationApp : public aie::Application {
public:

	_2018_03_04_PhysicsEngineDemonstrationApp();
	virtual ~_2018_03_04_PhysicsEngineDemonstrationApp();

	virtual bool startup();
	virtual void shutdown();

	virtual void update(float deltaTime);
	virtual void draw();

protected:

	Camera*		m_camera = nullptr;

	// camera transforms
	glm::mat4	m_viewMatrix;
	glm::mat4	m_projectionMatrix;

	// Physics
	Physebs::Scene*		m_scene = nullptr;

};