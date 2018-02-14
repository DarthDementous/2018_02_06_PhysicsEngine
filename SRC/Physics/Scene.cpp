#include "Physics\Scene.h"
#include "Physics\Rigidbody.h"
#include "Physics/Sphere.h"
#include <glm/ext.hpp>
#include <assert.h>

using namespace Physebs;

Scene::Scene(const glm::vec3 & a_gravityForce, const glm::vec3& a_globalForce) : m_gravity(a_gravityForce), m_globalForce(a_globalForce)
{
	// Defaults for 100fps
	m_fixedTimeStep		= 0.01f;		// One-hundreth of a second
	m_accumulatedTime	= 0.f;
}

Scene::~Scene()
{
	// Objects are under responsibility of the scene now, free their allocated memory
	for (auto obj : m_objects) {
		delete obj;
	}
}

/**
*	@brief Run update functionality within a fixed timestep.
*	@param a_dt is the actual time between frames.
*/
void Scene::FixedUpdate(float a_dt)
{
	m_accumulatedTime += a_dt;

	// Run update until no more extra time between updates left
	while (m_accumulatedTime >= m_fixedTimeStep) {

		Update();

		m_accumulatedTime -= m_fixedTimeStep;	// Account for time overflow
	}
}

void Scene::Update() {
	ApplyGravity();

	for (auto obj : m_objects) {
		obj->Update(m_fixedTimeStep);			// Regardless of how long update takes to be called, time between frames will be consistent now
	}

	// Detect and resolve collisions after calculating object movement
	DetectCollisions();
	ResolveCollisions();
}

void Scene::Draw()
{
	// Attach gizmos to objects
	for (auto obj : m_objects) {
		obj->Draw();
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

/**
*	@brief Apply defined force to every object in the scene.
*	@return void.
*/
void Scene::ApplyGlobalForce()
{
	for (auto obj : m_objects) {
		obj->ApplyForce(m_globalForce);
	}
}

/**
*	@brief Determine whether there is an overlap between two spheres.
*	NOTE: Static function means it can be used for utility purposes.
*	@param a_actor is the address to the sphere being collided with.
*	@param a_other is the address to the sphere that has collided with a_actor.
*	@return TRUE overlap between spheres || FALSE no overlap between spheres
*/
bool Scene::IsColliding_Sphere_Sphere(Sphere * a_actor, Sphere * a_other)
{
	// Distance between origins is less than radii of the spheres, they have collided
	if (glm::distance(a_actor->GetPos(), a_other->GetPos()) < a_actor->GetRadius() + a_other->GetRadius()) {
		return true;
	}
	

	// No overlapping detected
	return false;
}

/**
*	@brief Apply defined gravity force to every object in the scene.
*	@return void.
*/
void Scene::ApplyGravity()
{
	for (auto obj : m_objects) {
		obj->ApplyForce(m_gravity * obj->GetMass());	// Ensure heavier objects have a greater gravity force acting on them
	}
}

/**
*	@brief Check every object against every other object and record collisions for this frame.
*	@return void.
*/
void Scene::DetectCollisions()
{
	// Loop through all objects and check all actor objects against all other objects
	for (auto actor_iter = m_objects.begin(); actor_iter != m_objects.end(); ++actor_iter) {
		
		// Second for loop is shifted one to the right in order to access the 'others'
		for (auto other_iter = actor_iter + 1; other_iter != m_objects.end(); ++other_iter) {
			/// Sphere vs sphere
			Sphere* actor = static_cast<Sphere*>(*actor_iter);
			Sphere* other = static_cast<Sphere*>(*other_iter);

			if (actor != nullptr && other != nullptr) {
				
				if (IsColliding_Sphere_Sphere(actor, other)) {
					m_collisions.push_back(Collision(actor, other));
				}

			}
		}

	}
}

/**
*	@brief Apply impulse force to colliding objects to knock them back.
*	@return void.
*/
void Scene::ResolveCollisions() {
	// TODO: Replace simple but naive way of resolving collisions
	for (auto coll : m_collisions) {
		// TODO: Need to separate objects before knocking them back so they are no longer colliding

		// Determine knockback force from mass and velocity from perspective of each object. WARNING: Only works in one direction
		glm::vec3 knockbackForceA = coll.other->GetMass() * coll.other->GetVel();
		glm::vec3 knockbackForceB = coll.actor->GetMass() * coll.actor->GetVel();

		coll.actor->ApplyForce(knockbackForceA);
		coll.other->ApplyForce(knockbackForceB);

		/*
		1. Create a vector from A to B
		2. Create a normalised collision vector
		3. Calculate the relative velocity
		4. Use dot product to get projected length of relative velocity along collision vector

		Impulse Force:
		- Add restitution value to objects
		- Use resources formula to calculate j
		- Add ApplyImpulse function to object
		*/
	}

	// Collisions have been resolved for this frame, clear all recorded collisions
	m_collisions.clear();
}