#include "Physics\Scene.h"
#include "Physics\Rigidbody.h"
#include "Physics\Sphere.h"
#include "Physics\Plane.h"
#include "Physics\AABB.h"
#include "Physics\Constraint.h"
#include <glm/ext.hpp>
#include <assert.h>
#include <Gizmos.h>
#include <algorithm>
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

	// Free memory held by all constraints
	for (auto constraint : m_constraints) {
		delete constraint;
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

	for (auto constraint : m_constraints) {
		constraint->Update();
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

	// Represent constraints
	for (auto constraint : m_constraints) {
		constraint->Draw();
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
*	@brief Find and remove an object from the scene (do not delete memory, no longer responsible for object)
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
*	@brief Add a constraint to the scene to manage and enforce (already existing).
*	@param a_constraint is the constraint to add
*	@return void.
*/
void Scene::AddConstraint(Constraint * a_constraint)
{
	m_constraints.push_back(a_constraint);
}

/**
*	@brief Find and remove a constraint from the scene (do not delete memory, no longer responsible for constraint)
*	@param a_constraint is the constraint to remove.
*	@return void.
*/
void Scene::RemoveConstraint(Constraint * a_constraint)
{
	// Find corresponding iterator to constraint pointer and remove from vector
	auto foundIter = std::find(m_constraints.begin(), m_constraints.end(), a_constraint);

	assert(foundIter != m_constraints.end() && "Attempted to remove constraint from scene that it does not own.");

	m_constraints.erase(foundIter);
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
*	@param a_collision is the collision object reference to apply and get possible collision information from.
*	@return TRUE overlap between spheres || FALSE no overlap between spheres
*/
bool Scene::IsColliding_Sphere_Sphere(Collision& a_collision)
{
	assert(a_collision.actor->GetShape() == SPHERE && "IsColliding_Sphere_Sphere collision actor is not a sphere.");
	Sphere* actorSphere = static_cast<Sphere*>(a_collision.actor);

	assert(a_collision.other->GetShape() == SPHERE && "IsColliding_Sphere_Sphere collision other is not a sphere.");
	Sphere* otherSphere = static_cast<Sphere*>(a_collision.other);

	// Distance between origins is less than radii of the spheres, they have collided
	glm::vec3 collVec = otherSphere->GetPos() - actorSphere->GetPos();
	float dist = glm::length(collVec);

	if (dist < actorSphere->GetRadius() + otherSphere->GetRadius()) {
		a_collision.overlap			= (actorSphere->GetRadius() + otherSphere ->GetRadius()) - dist; //  (Radii - distance) NOT (distance - radii) or there will be negative cases
		a_collision.collisionNormal = (dist != 0) ? glm::normalize(collVec) : collVec;	// Normalize collision vector if length is not 0

		return true;
	}
	

	// No overlapping detected
	return false;
}

/**
*	@brief Determine whether there is an overlap between a sphere and a plane.
*	NOTE: Assumed actor is sphere and other is plane.
*	NOTE: USES REVERSE ISCOLLIDING FUNCTION AFTER SWAPPING ACTOR AND OTHER.
*	@param a_collision is the collision object reference to apply and get possible collision information from.
*	@return TRUE overlap between objects || FALSE no overlap between objects
*/
bool Scene::IsColliding_Sphere_Plane(Collision & a_collision)
{
	// Swap actor and other in order to utilise the Plane Vs Sphere function
	a_collision.SwapObjects();

	return IsColliding_Plane_Sphere(a_collision);
}

/**
*	@brief Determine whether there is an overlap between a sphere and an AABB.
*	NOTE: Assumed actor is sphere and other is AABB.
*	NOTE: USES REVERSE ISCOLLIDING FUNCTION AFTER SWAPPING ACTOR AND OTHER.
*	@param a_collision is the collision object reference to apply and get possible collision information from.
*	@return TRUE overlap between objects || FALSE no overlap between objects
*/
bool Scene::IsColliding_Sphere_AABB(Collision & a_collision)
{
	// Swap actor and other in order to utilise the AABB Vs Sphere function
	a_collision.SwapObjects();

	return IsColliding_AABB_Sphere(a_collision);
}

/**
*	@brief Determine whether there is an overlap between a plane and a sphere.
*	NOTE: Assumed actor is plane and other is sphere.
*	@param a_collision is the collision object reference to apply and get possible collision information from.
*	@return TRUE overlap between objects || FALSE no overlap between objects
*/
bool Scene::IsColliding_Plane_Sphere(Collision& a_collision) {
	assert(a_collision.actor->GetShape() == PLANE && "IsColliding_Plane_Sphere collision actor is not a plane.");
	Plane*	actorPlane	= static_cast<Plane*>(a_collision.actor);

	assert(a_collision.other->GetShape() == SPHERE && "IsColliding_Plane_Sphere collision other is not a sphere.");
	Sphere*	otherSphere = static_cast<Sphere*>(a_collision.other);

	/// Assume everything is at the origin so positions can also be vectors
	// 1. Dot-product plane normal with sphere position (aka vector from origin to sphere position) = distance between sphere position and plane
	float sphereDist = glm::dot(actorPlane->GetNormal(), otherSphere->GetPos());
	// 2. Get final distance by minusing plane distance from sphere distance to account for plane not being at the origin
	float finalDist = sphereDist - actorPlane->GetDist();

	// 4. Check if final distance is less than the radius of the sphere
	if (finalDist < otherSphere->GetRadius()) {
		a_collision.overlap				= otherSphere->GetRadius() - finalDist;		// How much objects have intersected taking sphere radius into account
		a_collision.collisionNormal		= actorPlane->GetNormal();

		return true;
	}

	// No overlapping detected
	return false;
}

/**
*	@brief Determine whether there is an overlap between a plane and a AABB.
*	NOTE: Assumed actor is plane and other is AABB.
*	@param a_collision is the collision object reference to apply and get possible collision information from.
*	@return TRUE overlap between objects || FALSE no overlap between objects
*/
bool Scene::IsColliding_Plane_AABB(Collision& a_collision) {
	assert(a_collision.actor->GetShape() == PLANE && "IsColliding_Plane_Sphere collision actor is not a plane.");
	Plane*	actorPlane = static_cast<Plane*>(a_collision.actor);

	assert(a_collision.other->GetShape() == AA_BOX && "IsColliding_Plane_Sphere collision other is not an AABB.");
	AABB*	otherAABB = static_cast<AABB*>(a_collision.other);

	/// Assume everything is at the origin so positions can also be vectors
	// 1. Dot-product plane normal with AABB min (aka vector from origin to AABB min) = distance between AABB and plane
	float AABBDist = glm::dot(actorPlane->GetNormal(), otherAABB->CalculateMin());
	// 2. Because assuming objects are at origin, minus with plane distance from origin for accurate distance calculation
	AABBDist -= actorPlane->GetDist();

	// 2. If distance is 0 or negative then AABB is colliding
	if (AABBDist <= 0) {
		// Distance is calculated from edge so it can be used as overlap. Abs because overlap is never negative
		a_collision.overlap = abs(AABBDist);		
		a_collision.collisionNormal = actorPlane->GetNormal();

		return true;
	}

	// No overlapping detected
	return false;
}

/**
*	@brief Determine whether there is an overlap between an AABB and a plane.
*	NOTE: Assumed actor is AABB and other is Plane.
*	NOTE: USES REVERSE ISCOLLIDING FUNCTION AFTER SWAPPING ACTOR AND OTHER.
*	@param a_collision is the collision object reference to apply and get possible collision information from.
*	@return TRUE overlap between objects || FALSE no overlap between objects
*/
bool Scene::IsColliding_AABB_Plane(Collision & a_collision)
{
	// Swap actor and other in order to utilise the Plane Vs AABB function
	a_collision.SwapObjects();

	return IsColliding_Plane_AABB(a_collision);
}

/**
*	@brief Determine whether there is an overlap between an AABB and a sphere.
*	NOTE: Assumed actor is an AABB and other is a sphere.
*	@param a_collision is the collision object reference to apply and get possible collision information from.
*	@return TRUE overlap between objects || FALSE no overlap between objects
*/
bool Scene::IsColliding_AABB_Sphere(Collision& a_collision) {
	assert(a_collision.actor->GetShape() == AA_BOX && "IsColliding_AABB_Sphere collision actor is not an AABB.");
	AABB*	actorAABB = static_cast<AABB*>(a_collision.actor);

	assert(a_collision.other->GetShape() == SPHERE && "IsColliding_Plane_Sphere collision other is not a sphere.");
	Sphere*	otherSphere = static_cast<Sphere*>(a_collision.other);

	// Save temporary values into vec3 copy so glm::clamp doesn't use volatile memory
	glm::vec3 AABBMin = actorAABB->CalculateMin();
	glm::vec3 AABBMax = actorAABB->CalculateMax();

	// 1. Get closest point on AABB by clamping sphere position between AABB min and max
	glm::vec3 closestPoint = glm::clamp(otherSphere->GetPos(), AABBMin, AABBMax);
	// 2. Create vector from sphere to closest point on AABB
	glm::vec3 sphereToClosestPoint = otherSphere->GetPos() - closestPoint;		// [B - A]
	// 3. Get distance between sphere position and AABB by finding length of vector between sphere position and closest point
	float dist = glm::length(sphereToClosestPoint);

	// 4. Check if distance is less than the radius of the sphere
	if (dist < otherSphere->GetRadius()) {
		a_collision.overlap = otherSphere->GetRadius() - dist;		// How much objects have intersected taking sphere radius into account

		// Get collision normal from vector between sphere and closest point and normalize if length isn't 0 to avoid nan errors
		a_collision.collisionNormal = dist != 0 ? glm::normalize(sphereToClosestPoint) : sphereToClosestPoint;

		return true;
	}

	return false;
}

/**
*	@brief Determine whether there is an overlap between an AABB and an AABB.
*	NOTE: Assumed actor is an AABB and other is an AABB.
*	@param a_collision is the collision object reference to apply and get possible collision information from.
*	@return TRUE overlap between objects || FALSE no overlap between objects
*/
bool Scene::IsColliding_AABB_AABB(Collision & a_collision)
{
	assert(a_collision.actor->GetShape() == AA_BOX && "IsColliding_AABB_AABB collision actor is not an AABB.");
	AABB*	actorAABB = static_cast<AABB*>(a_collision.actor);

	assert(a_collision.other->GetShape() == AA_BOX && "IsColliding_AABB_AABB collision other is not a plane.");
	AABB*	otherAABB = static_cast<AABB*>(a_collision.other);

	glm::vec3 actorMin = actorAABB->CalculateMin();
	glm::vec3 actorMax = actorAABB->CalculateMax();

	glm::vec3 otherMin = otherAABB->CalculateMin();
	glm::vec3 otherMax = otherAABB->CalculateMax();

	// Check if AABB bounding volumes are overlapping
	if (actorMin.x < otherMax.x && actorMax.x > otherMin.x &&
		actorMin.y < otherMax.y && actorMax.y > otherMin.y &&
		actorMin.z < otherMax.z && actorMax.z > otherMin.z)
	{
		

#if 1
		/// THIS METHOD WORKS FOR ANY KIND OF COLLISION SCENARIO
		// Calculate overlap between AABBs by finding distance between closest point on actor and closest point on other.
		// Closest point is found by clamping opposite AABB position by AABB's min and max
		glm::vec3 actorClosestPoint = glm::clamp(otherAABB->GetPos(), actorMin, actorMax);
		glm::vec3 otherClosestPoint = glm::clamp(actorAABB->GetPos(), otherMin, otherMax);
		glm::vec3 overlapVec		= otherClosestPoint - actorClosestPoint;

		a_collision.overlap = glm::length(overlapVec);
		
		//// B - A to get collision vector for consistency
		glm::vec3 collVec = otherAABB->GetPos() - actorAABB->GetPos();

		// Get collision normal by reflecting velocity
		//glm::vec3 collVec				= -otherAABB->GetVel();

		a_collision.collisionNormal		= glm::length(collVec) != 0 ? glm::normalize(collVec) : collVec;	// Normalize collision vector if length is not 0#else 

#else
		/// THIS METHOD IS ONLY ACCURATE IF ACTOR IS ON THE LEFT AND OTHER IS ON THE RIGHT
		// Calculate overlap between AABBs on each axis by using actor max and other min. 
		//NOTE: Use abs so direction of created vector doesn't matter and ensure that overlap is never negative.
		float xOverlap = abs((actorMax - otherMin).x);
		float yOverlap = abs((actorMax - otherMin).y);
		float zOverlap = abs((actorMax - otherMin).z);

		glm::vec3 overlapVec(xOverlap, yOverlap, zOverlap);

		/*	Convert overlap vector into a total overlap by dot-producting with collision normal to ensure overlap is applied based off 
			the direction of the collision.

				E.g. if an AABB needs to move (0.1, 5, 7) to no longer be overlapping and one object approaches from the top 
				(0, 1, 0) then
					totalOverlap = (0.1, 5, 7) . (0, 1, 0)
					totalOverlap = (0.1 * 0 + 5 * 1 + 7 * 0)
					totalOverlap = 5

				Therefore both objects will be moved back 2.5 units along the collision normal to stop them from overlapping
				because the direction of collision dictates that only the y axis overlap should be considered.
		*/
		a_collision.overlap = abs(glm::dot(overlapVec, a_collision.collisionNormal));	// Abs because overlap must never be negative
#endif
		return true;
	}

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
			
			// For each set of checks, create a temporary collision object to pass into colliding check functions
			Collision tempCollision(actor, other);

#pragma region Object Type Detection and Collision Checks
			// Determine what kind of object collision we're checking for and detect appropriately
			switch (actor->GetShape())
			{
				case SPHERE:
				{
					switch (other->GetShape())
					{
						case SPHERE:
						{
							if (IsColliding_Sphere_Sphere(tempCollision)) {
								m_collisions.push_back(tempCollision);
							}

							break;
						}
						case AA_BOX:
						{
							if (IsColliding_Sphere_AABB(tempCollision)) {
								m_collisions.push_back(tempCollision);
							}
							break;
						}
						case PLANE:
						{
							if (IsColliding_Sphere_Plane(tempCollision)) {
								m_collisions.push_back(tempCollision);
							}

							break;
						}
					}
					break;
				}
				case PLANE:
				{
					switch (other->GetShape())
					{
						case SPHERE:
						{
							if (IsColliding_Plane_Sphere(tempCollision)) {
								m_collisions.push_back(tempCollision);
							}

							break;
						}
						case AA_BOX:
						{
							if (IsColliding_Plane_AABB(tempCollision)) {
								m_collisions.push_back(tempCollision);
							}
							break;
						}
					}
					break;
				}
				case AA_BOX:
				{
					switch (other->GetShape())
					{
						case SPHERE:
						{
							if (IsColliding_AABB_Sphere(tempCollision)) {
								m_collisions.push_back(tempCollision);
							}

							break;
						}
						case AA_BOX:
						{
							if (IsColliding_AABB_AABB(tempCollision)) {
								m_collisions.push_back(tempCollision);
							}
							break;
						}
						case PLANE:
						{
							if (IsColliding_AABB_Plane(tempCollision)) {
								m_collisions.push_back(tempCollision);
							}
							break;
						}
					}
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
		// Objects are not completely static (collision vec length is not 0)
		if (glm::length(coll.collisionNormal) != 0) {
			// Use half of the overlap value (half push-back for each object) to set their position back along the appropriate collision normal so objects are no longer touching
			glm::vec3 overlapAccountVec = (coll.overlap / 2) * coll.collisionNormal;

			/// DYNAMIC ACTOR, STATIC OTHER
			if (coll.actor->GetIsDynamic() && !(coll.other->GetIsDynamic())) {
				// Account for overlap for actor
				coll.actor->SetPos(coll.actor->GetPos() + (-overlapAccountVec));		// Modifying actor, make overlap vec negative

				ApplyKnockback_Static(coll);
			}
			/// STATIC ACTOR, DYNAMIC OTHER
			if (!(coll.actor->GetIsDynamic()) && coll.other->GetIsDynamic()) {
				// Account for overlap for other
				coll.other->SetPos(coll.other->GetPos() + overlapAccountVec);

				ApplyKnockback_Static(coll);
			}
			/// DYNAMIC ACTOR, DYNAMIC OTHER
			if (coll.actor->GetIsDynamic() && coll.other->GetIsDynamic()) {
				// Account for overlap for actor and other (actor is negative because collisionNormal created from B - A)
				coll.actor->SetPos(coll.actor->GetPos() + -overlapAccountVec);
				coll.other->SetPos(coll.other->GetPos() + overlapAccountVec);

				ApplyKnockback_Dynamic(coll);
			}
		}

		
	}

	// Collisions have been resolved for this frame, clear all recorded collisions
	m_collisions.clear();
}

/**
*	@brief Calculate and apply impulse knockback taking the relative velocity of the two objects into account.
*	NOTE: Should only be used if both actor and other are dynamic rigidbodies.
*	@param a_collision is the collision object reference to apply and get possible collision information from.
*	@return void.
*/
void Scene::ApplyKnockback_Dynamic(Collision& a_collision)
{
	const float restitution = 0.5f;	// TODO: Place restitution variable into Rigidbody

	/// Use collision resolution equation to find scale of impulse knockback force
	// Modify the ratio of impulse force (MUST USE SAME METHOD AS CALCULATING COLLISION NORMAL: [B - A])
	glm::vec3 relativeVel = a_collision.other->GetVel() - a_collision.actor->GetVel();

	float impulseScale =
		glm::dot(-(1 + restitution) * relativeVel, a_collision.collisionNormal) /
		glm::dot(a_collision.collisionNormal, a_collision.collisionNormal * (1 / a_collision.actor->GetMass() + 1 / a_collision.other->GetMass()));

	// Apply impulse resolution force along appropriate direction for each object (assumes collision normal is pointing from actor to other [B - A])
	a_collision.actor->ApplyImpulseForce(impulseScale * -a_collision.collisionNormal);
	a_collision.other->ApplyImpulseForce(impulseScale * a_collision.collisionNormal);
}

/**
*	@brief Calculate and apply impulse knockback taking only the velocity of the dynamic object
*	NOTE: Should only be used if one of the rigidbodies are static.
*	@param a_collision is the collision object reference to apply and get possible collision information from.
*	@return void.
*/
void Scene::ApplyKnockback_Static(Collision& a_collision)
{
	const float restitution = 0.5f;

	// For readability create separate pointers that distinguish which object is static and which object is dynamic
	Rigidbody* staticObj;
	Rigidbody* dynamicObj;

	// Assume that there's only one static object
	if (a_collision.actor->GetIsDynamic()) {
		dynamicObj	= a_collision.actor;
		staticObj	= a_collision.other;
	}
	else {
		dynamicObj	= a_collision.other;
		staticObj	= a_collision.actor;
	}

	// Use collision resolution equation to find impulse knockback force.
	// NOTE: Because one object is static, assume it has infinite mass and that the relative velocity is the velocity of the dynamic object
	float impulseScale =
		glm::dot(-(1 + restitution) * dynamicObj->GetVel(), a_collision.collisionNormal) /
		(1 / dynamicObj->GetMass());

	// Apply impulse resolution force along collision normal for dynamic object
	dynamicObj->ApplyImpulseForce(impulseScale * a_collision.collisionNormal);
}
