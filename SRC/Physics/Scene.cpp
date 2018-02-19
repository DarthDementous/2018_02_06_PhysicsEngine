#include "Physics\Scene.h"
#include "Physics\Rigidbody.h"
#include "Physics\Sphere.h"
#include "Physics\Plane.h"
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
*	@brief Determine whether there is an overlap between a plane and a sphere.
*	@param a_actor is the address to the plane being collided with.
*	@param a_other is the address to the sphere that has collided with a_actor.
*	@param a_overlapRef is the overlap value passed in by reference that will be assigned the overlap value of the collision.
*	@return TRUE overlap between objects || FALSE no overlap between spheres
*/
bool Scene::IsColliding_Plane_Sphere(Plane* a_actor, Sphere* a_other, float& a_overlapRef) {
	/// Assume everything is at the origin so positions can also be vectors
	// 1. Dot-product plane normal with sphere position (aka vector from origin to sphere position) = distance between sphere position and plane
	float overlap = glm::dot(a_actor->GetNormal(), a_other->GetPos());
	// 2. Get length of vector between plane position and origin (aka plane position)
	float distPlane = glm::length(a_actor->GetPos());
	// 3. Get final overlap by minusing plane distance from sphere distance to to put sphere in same space as the plane
	overlap -= distPlane;

	// Assign overlap to overlap reference float
	a_overlapRef = overlap;

	// 4. Check if overlap is less than the radius of the sphere
	if (overlap < a_other->GetRadius()) {
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
			Rigidbody* actor = *actor_iter;
			Rigidbody* other = *other_iter;
			
			// How much the objects overlap in total
			float overlap;

			// Determine what kind of object collision we're checking for and detect appropriately
			switch (actor->GetShape()) {
				
			case SPHERE :
				// Static cast to access sphere functions
				Sphere * actorSphere = static_cast<Sphere*>(actor);

					switch (other->GetShape()) {

					case SPHERE :
						Sphere* otherSphere = static_cast<Sphere*>(other);

						// Find overlap by taking radii of spheres into account between distance of origins (Radii - distance) NOT (distance - radii) or there will be negative cases
						overlap = (actorSphere->GetRadius() + otherSphere->GetRadius()) - glm::distance(actorSphere->GetPos(), otherSphere->GetPos());

						m_collisions.push_back(Collision(actorSphere, otherSphere, overlap));
						break;

					case PLANE:
						Plane * otherPlane = static_cast<Plane*>(other);


					}
			}
			float overlap = 0.f;		// How much shapes overlap each other (for collision resolution)
			
			if (actorSphere != nullptr) {
				/// Sphere vs sphere
				if (otherSphere != nullptr) {

				}
			}

			if (actorSphere != nullptr && otherSphere != nullptr) {
				
				if (IsColliding_Sphere_Sphere(actorSphere, otherSphere)) {
					// Find overlap by taking radii of spheres into account between distance of origins (Radii - distance) NOT (distance - radii) or there will be negative cases
					overlap = (actorSphere->GetRadius() + otherSphere->GetRadius()) - glm::distance(actorSphere->GetPos(), otherSphere->GetPos());

					m_collisions.push_back(Collision(actorSphere, otherSphere, overlap));
				}

			}

			/// Plane vs sphere
			if (actorPlane != nullptr && otherSphere != nullptr)
		}

	}
}

/**
*	@brief Apply impulse force to colliding objects to knock them back.
*	@return void.
*/
void Scene::ResolveCollisions() {
	// TODO: Replace simple but naive way of resolving collisions
	static float restitution = 0.5f;	// TODO: Place restitution variable into Rigidbody

	for (auto coll : m_collisions) {
		// TODO: Need to separate objects before knocking them back so they are no longer colliding

		//// Determine knockback force from mass and velocity from perspective of each object. WARNING: Only works in one direction
		//glm::vec3 knockbackForceA = coll.other->GetMass() * coll.other->GetVel();
		//glm::vec3 knockbackForceB = coll.actor->GetMass() * coll.actor->GetVel();

		//coll.actor->ApplyForce(knockbackForceA);
		//coll.other->ApplyForce(knockbackForceB);

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
		/// VECTORS MUST ALWAYS POINT FROM OBJECT A TO OBJECT B (B - A)
		// What direction to apply impulse knockback force to the objects in [B - A]. A WILL GET NEGATIVE, B WILL GET POSITIVE FORCE TO PUSH BACK
		glm::vec3 collisionNormal = coll.other->GetPos() - coll.actor->GetPos();				

		// Objects are not completely static (collision vec length means not dividing by 0)
		if (glm::length(collisionNormal) != 0) {
			collisionNormal = glm::normalize(collisionNormal);

			/// Use half of the overlap value (half push-back for each object) to set their position back along the appropriate collision normal so objects are no longer touching
			// Actor isn't static, allow position to be modified to account for overlap
			if (coll.actor->GetIsDynamic()) {
				
				coll.actor->SetPos(coll.actor->GetPos() + ((coll.overlap / 2) * -collisionNormal));
			}
			// Other object isn't static, allow position to be modified to account for overlap
			if (coll.other->GetIsDynamic()) {

				coll.other->SetPos(coll.other->GetPos() + ((coll.overlap / 2) * collisionNormal));
			}

			/// Use collision resolution equation to find scale of impulse knockback force
			// Modify the ratio of impulse force (MUST USE SAME METHOD AS CALCULATING COLLISION NORMAL: [B - A])
			glm::vec3 relativeVel = coll.other->GetVel() - coll.actor->GetVel();

			float impulseScale =
				glm::dot(-(1 + restitution) * relativeVel, collisionNormal) /
				glm::dot(collisionNormal, collisionNormal * (1 / coll.actor->GetMass() + 1 / coll.other->GetMass()));

			// Apply impulse resolution force along appropriate direction for each object
			coll.actor->ApplyImpulseForce(impulseScale * -collisionNormal);
			coll.other->ApplyImpulseForce(impulseScale * collisionNormal);
		}

		
	}

	// Collisions have been resolved for this frame, clear all recorded collisions
	m_collisions.clear();
}