#include "Physics\Scene.h"
#include "Physics\Rigidbody.h"
#include <assert.h>

using namespace Physebs;

Scene::Scene()
{
}

Scene::~Scene()
{
	// Objects are under responsibility of the scene now, free their allocated memory
	for (auto obj : m_objects) {
		delete obj;
	}
}

void Scene::Update(float a_dt)
{
	for (auto obj : m_objects) {
		obj->Update(a_dt);
	}
}

/**
*	@brief Add an object for the scene to manage (already existing).
*	@param a_obj is the object to add.
*	@return void.
*/
void Scene::AddObject(Rigidbody * a_obj)
{
	m_objects.push_back(a_obj);
}

/**
*	@brief Remove an object from the scene (do not delete it).
*	@param a_obj is the object to remove.
*	@return void.
*/
void Scene::RemoveObject(Rigidbody * a_obj)
{
	// Find corresponding iterator to object pointer and remove from vector
	auto foundIter = std::find(m_objects.begin(), m_objects.end(), a_obj);

	assert(foundIter != m_objects.end() && "Attempted to remove object from scene that it does not own.");

	m_objects.erase(foundIter);
}
