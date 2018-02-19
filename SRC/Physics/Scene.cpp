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

#pragma region Object Type Detection and Collision Checks
			// Determine what kind of object collision we're checking for and detect appropriately
			switch (actor->GetShape())
			{
			case SPHERE:
			{
				// Static cast to access sphere functions
				Sphere * actorSphere = static_cast<Sphere*>(actor);

				switch (other->GetShape())
				{
				case SPHERE:
				{
					Sphere* otherSphere = static_cast<Sphere*>(other);

					// Find overlap by taking radii of spheres into account between distance of origins (Radii - distance) NOT (distance - radii) or there will be negative cases
					overlap = (actorSphere->GetRadius() + otherSphere->GetRadius()) - glm::distance(actorSphere->GetPos(), otherSphere->GetPos());

					m_collisions.push_back(Collision(actorSphere, otherSphere, overlap));
					break;
				}
				case AABB:
				{
					// TODO: Write code for sphere vs AABB collision check
					break;
				}
				case PLANE:
				{
					Plane * otherPlane = static_cast<Plane*>(other);

					if (IsColliding_Plane_Sphere(otherPlane, actorSphere, overlap)) {
						m_collisions.push_back(Collision(actorSphere, otherPlane, overlap));
					}

					break;
				}
				}
				break;
			}
			case PLANE:
			{
				Plane* actorPlane = static_cast<Plane*>(actor);

				switch (other->GetShape())
				{
				case SPHERE:
				{
					Sphere* otherSphere = static_cast<Sphere*>(other);

					if (IsColliding_Plane_Sphere(actorPlane, otherSphere, overlap)) {
						m_collisions.push_back(Collision(actorPlane, otherSphere, overlap));
					}

					break;
				}
				case AABB:
				{
					// TODO: Write code for plane vs AABB collision check
					break;
				}
				}
				break;
			}
			}
		}
#pragma endregion
	
	}
}

/**
*	@brief Apply impulse force to colliding objects to knock them back.
*	@return void.
*/
void Scene::ResolveCollisions() {
	// TODO: Replace simple but naive way of resolving collisions

	for (auto coll : m_collisions) {
		/// VECTORS MUST ALWAYS POINT FROM OBJECT A TO OBJECT B (B - A)
		// What direction to apply impulse knockback force to the objects in [B - A]. A WILL GET NEGATIVE, B WILL GET POSITIVE FORCE TO PUSH BACK
		glm::vec3 collisionNormal = coll.other->GetPos() - coll.actor->GetPos();

		// Objects are not completely static (collision vec length means not dividing by 0)
		if (glm::length(collisionNormal) != 0) {
			collisionNormal = glm::normalize(collisionNormal);

			// Use half of the overlap value (half push-back for each object) to set their position back along the appropriate collision normal so objects are no longer touching
			glm::vec3 overlapAccountVec = (coll.overlap / 2) * collisionNormal;

			/// DYNAMIC ACTOR, STATIC OTHER
			if (coll.actor->GetIsDynamic() && !(coll.other->GetIsDynamic())) {
				// Account for overlap for actor
				coll.actor->SetPos(coll.actor->GetPos() + overlapAccountVec);

				ApplyKnockback_Static(coll.actor, coll.other, collisionNormal);
			}
			/// STATIC ACTOR, DYNAMIC OTHER
			if (!(coll.actor->GetIsDynamic()) && coll.other->GetIsDynamic()) {
				// Account for overlap for other
				coll.other->SetPos(coll.other->GetPos() + overlapAccountVec);

				ApplyKnockback_Static(coll.actor, coll.other, collisionNormal);
			}
			/// DYNAMIC ACTOR, DYNAMIC OTHER
			if (coll.actor->GetIsDynamic() && coll.other->GetIsDynamic()) {
				// Account for overlap for actor and other (actor is negative because collisionNormal created from B - A)
				coll.actor->SetPos(coll.actor->GetPos() + -overlapAccountVec);
				coll.other->SetPos(coll.other->GetPos() + overlapAccountVec);

				ApplyKnockback_Dynamic(coll.actor, coll.other, collisionNormal);
			}
		}

		
	}

	// Collisions have been resolved for this frame, clear all recorded collisions
	m_collisions.clear();
}

/**
*	@brief Calculate and apply impulse knockback taking the relative velocity of the two objects into account.
*	NOTE: Should only be used if both actor and other are dynamic rigidbodies.
*	@param a_actor is the pointer to the actor object.
*	@param a_other is the pointer to the other object.
*	@param a_collisionNormal is the vector used to determine the direction to push each object back in. (ACTOR RECEIVES NEGATIVE OF THIS BY DEFAULT).
*	@return void.
*/
void Scene::ApplyKnockback_Dynamic(Rigidbody * a_actor, Rigidbody * a_other, const glm::vec3& a_collisionNormal)
{
	const float restitution = 0.5f;	// TODO: Place restitution variable into Rigidbody

	/// Use collision resolution equation to find scale of impulse knockback force
	// Modify the ratio of impulse force (MUST USE SAME METHOD AS CALCULATING COLLISION NORMAL: [B - A])
	glm::vec3 relativeVel = a_other->GetVel() - a_actor->GetVel();

	float impulseScale =
		glm::dot(-(1 + restitution) * relativeVel, a_collisionNormal) /
		glm::dot(a_collisionNormal, a_collisionNormal * (1 / a_actor->GetMass() + 1 / a_other->GetMass()));

	// Apply impulse resolution force along appropriate direction for each object
	a_actor->ApplyImpulseForce(impulseScale * -a_collisionNormal);
	a_other->ApplyImpulseForce(impulseScale * a_collisionNormal);
}

/**
*	@brief Calculate and apply impulse knockback taking only the velocity of the dynamic object
*	NOTE: Should only be used if one of the rigidbodies are static.
*	@param a_actor is the pointer to the actor object.
*	@param a_other is the pointer to the other object.
*	@param a_collisionNormal is the direction to push the dynamic object in.
*	@return void.
*/
void Physebs::Scene::ApplyKnockback_Static(Rigidbody * a_actor, Rigidbody * a_other, const glm::vec3& a_collisionNormal)
{
	const float restitution = 0.5f;

	// For readability create separate pointers that distinguish which object is static and which object is dynamic
	Rigidbody* staticObj;
	Rigidbody* dynamicObj;

	// Assume that there's only one static object
	if (a_actor->GetIsDynamic()) {
		dynamicObj	= a_actor;
		staticObj	= a_other;
	}
	else {
		dynamicObj	= a_other;
		staticObj	= a_actor;
	}

	// Use collision resolution equation to find impulse knockback force.
	// NOTE: Because one object is static, assume it has infinite mass and that the relative velocity is the velocity of the dynamic object
	float impulseScale =
		glm::dot(-(1 + restitution) * dynamicObj->GetVel(), a_collisionNormal) /
		1 / dynamicObj->GetMass();

	// Apply impulse resolution force along collision normal for dynamic object
	dynamicObj->ApplyImpulseForce(impulseScale * a_collisionNormal);
}
