#include "Physics\Scene.h"
#include "Physics\Rigidbody.h"
#include "Physics\Sphere.h"
#include "Physics\Plane.h"
#include "Physics\AABB.h"
#include "Physics\Constraint.h"
#include "Octree\Octree.h"
#include <glm/ext.hpp>
#include <assert.h>
#include <algorithm>
#include <random>
using namespace Physebs;

Scene::Scene(const glm::vec3 & a_gravityForce, const glm::vec3& a_globalForce, 
	const glm::vec3& a_simulationOrigin, const glm::vec3& a_simulationHalfExtents) 
	: 
	m_gravity(a_gravityForce), m_globalForce(a_globalForce)
{
	// Defaults for 100fps
	m_fixedTimeStep		= 0.01f;		// One-hundreth of a second
	m_accumulatedTime	= 0.f;

	// Initialise spatial partition tree from given simulation origin and extents
	float simulationMin[3]	= { a_simulationOrigin.x - a_simulationHalfExtents.x, a_simulationOrigin.y - a_simulationHalfExtents.y, a_simulationOrigin.z - a_simulationHalfExtents.z };
	float simulationMax[3]	= { a_simulationOrigin.x + a_simulationHalfExtents.x, a_simulationOrigin.y + a_simulationHalfExtents.y, a_simulationOrigin.z + a_simulationHalfExtents.z };
	float minCellSize[3]	= MIN_VOLUME_SIZE;

	m_spatialPartitionTree = new Octree<PartitionNode>(simulationMin, simulationMax, minCellSize);
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

	// Free memory held by partition tree
	delete m_spatialPartitionTree;
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

	// Regardless of how long update takes to be called, time between frames will be consistent now
	for (auto obj : m_objects) {
		obj->Update(m_fixedTimeStep);			
	}

	for (auto constraint : m_constraints) {
		constraint->Update();
	}

	// Detect and resolve collisions after calculating object movement
	/// Octree optimisation, segments objects into volumes and checks only objects in that volume
	if (b_partitionCollisions) {
		PartitionCollisions();
	}

	/// O(n^2) complexity, checks every single object in scene against every other object
	else {
		// In case partitioning is switched off in real-time make sure partition tree is clear
		m_spatialPartitionTree->clear();

		DetectCollisions(m_objects);
	}

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

	// Draw AABB Gizmos to represent partition volumes
#if B_SHOW_PARTITIONS
	OctreeCallbackDebug ocd;

	m_spatialPartitionTree->traverse(&ocd);
#endif
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

	// If connected via constraint, remove and delete attached constraint
	for (auto constraint : m_constraints) {

		if (constraint->ContainsObj(a_obj)) {

			RemoveConstraint(constraint);
			delete constraint;
		}
	}
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
*	@brief Use all object positions to build octree with collision volumes within the simulation boundaries and segment collision detection between those volumes for 400% more efficiency.
*	NOTE: If an object is outside of the simulation boundaries it will be removed AND deleted.
*	detect collisions between each set of objects in their respective collision volumes.
*/
void Scene::PartitionCollisions()
{
	// Refresh volumes in tree to account for change in object positions
	m_spatialPartitionTree->clear();

	// Calculate volumes to encompass object positions and add corresponding object pointers to 'segment' the scene
	for (auto currentObj : m_objects) {
		const float objPos[3] = { currentObj->GetPos().x, currentObj->GetPos().y, currentObj->GetPos().z };

		// Object is outside of simulation extents, remove it from scene as well as delete it, and continue to next one to avoid accessing deleted memory
		if (!AABB::PointInMinMax(objPos, m_spatialPartitionTree->GetMin(), m_spatialPartitionTree->GetMax())) {

			RemoveObject(currentObj);
			delete currentObj;

			continue;
		}

		else {
			// Get node in the volume that contains object position and add object to node
			PartitionNode& n = m_spatialPartitionTree->getCell(objPos);
			n.containedObjects.push_back(currentObj);

			// Add scene pointer to node so detect collisions function can be called later
			n.scene = this;

			/// Change object colors to reflect what volume they are in
#if B_VOLUME_COLORS 
			// Assign object to volume color
			currentObj->SetColor(n.debugColor);
#endif
		}

	}

	// Traverse created volumes and use callback class to detect collisions only with objects contained in the volume
	OctreeCallbackDetectCollisions collBack;		// Puns are great, shhh
	m_spatialPartitionTree->traverse(&collBack);
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
	
	// Check if distance between origins is less than radii of the spheres, if so then they have collided
	glm::vec3 collVec = otherSphere->GetPos() - actorSphere->GetPos();
	float dist = glm::length(collVec);
	
	if (dist < actorSphere->GetRadius() + otherSphere->GetRadius()) {
		a_collision.overlap = (actorSphere->GetRadius() + otherSphere->GetRadius()) - dist;		//  (Radii - distance) NOT (distance - radii) or there will be negative cases
		a_collision.collisionNormal = (dist != 0) ? glm::normalize(collVec) : collVec;			// Normalize collision vector if length is not 0
	
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
	Plane*	actorPlane = static_cast<Plane*>(a_collision.actor);

	assert(a_collision.other->GetShape() == SPHERE && "IsColliding_Plane_Sphere collision other is not a sphere.");
	Sphere*	otherSphere = static_cast<Sphere*>(a_collision.other);

	/// Assume everything is at the origin so positions can also be vectors
	// Determine plane normal by dot-producting with sphere position to work out what side of the plane the sphere is on [positive = normal side, negative = other side]
	float		planeSideCheck	= glm::dot(actorPlane->GetNormal(), otherSphere->GetPos()) - actorPlane->GetDist();		// Account for offset from origin
	bool		b_otherSide		= planeSideCheck < 0 ? true : false;

	glm::vec3	planeNormal	= !b_otherSide ? actorPlane->GetNormal() : -actorPlane->GetNormal();
	float		planeDist	= !b_otherSide ? actorPlane->GetDist() : -actorPlane->GetDist();								// If on other side, distance from origin is negated to account for change in normal.
		
	// 1. Dot-product plane normal with sphere position (aka vector from origin to sphere position) = distance between sphere position and plane
	float sphereDist = glm::dot(planeNormal, otherSphere->GetPos());
	// 2. Get final distance by minusing plane distance from sphere distance to account for plane not being at the origin
	float finalDist = sphereDist - planeDist;

	// 3. Check if final distance is less than the radius of the sphere
	if (finalDist < otherSphere->GetRadius()) {
		a_collision.overlap = otherSphere->GetRadius() - finalDist;		// How much objects have intersected taking sphere radius into account
		a_collision.collisionNormal = planeNormal;

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
	// Determine plane normal by dot-producting with AABB position to work out what side of the plane the AABB is on [positive = normal side, negative = other side]
	float	planeSideCheck	= glm::dot(actorPlane->GetNormal(), otherAABB->GetPos()) - actorPlane->GetDist();	// Account for offset from origin
	bool	b_otherSide		= planeSideCheck < 0 ? true : false;

	glm::vec3 planeNormal	= !b_otherSide ? actorPlane->GetNormal() : -actorPlane->GetNormal();
	float planeDist			= !b_otherSide ? actorPlane->GetDist() : -actorPlane->GetDist();		// If on other side, distance from origin is negated to account for change in normal.

	// Loop through all AABB corners and:
	// - Find closest corner to plane distance
	auto		corners				= otherAABB->CalculateCorners();
	float		closestCornerDist	= glm::dot(planeNormal, corners[0]) - planeDist;	 // Set to distance from first corner

	for (int i = 0; i < corners.size(); ++i) {
		
		// 1. Dot-product plane normal with AABB corner (aka vector from origin to AABB corner) = distance between AABB and plane
		float AABBDist = glm::dot(planeNormal, corners[i]);

		// 2. Because assuming objects are at origin, minus with plane distance from origin for accurate distance calculation.
		AABBDist -= planeDist;

		// 3. Check if found new minimum distance and set new closest corner
		if (AABBDist < closestCornerDist) {

			closestCornerDist	= AABBDist;
		}

	}

	// 4. Check if distance between closest corner distance indicates intersection with plane (negative or 0 because plane has no thickness)
	if (closestCornerDist <= 0) {

		// Overlap is the straight distance between plane and closest corner (abs because overlap can never be negative)
		a_collision.overlap = abs(closestCornerDist);

		// Collision normal determined by what side of the plane AABB is on
		a_collision.collisionNormal = planeNormal;

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
		
		/// THIS METHOD WORKS FOR ANY KIND OF COLLISION SCENARIO
		// Calculate overlap between AABBs by minusing distance between AABBs from the total overlap volume
		// B - A to get vector between objects for consistency
		glm::vec3 betweenVec			= glm::abs(otherAABB->GetPos() - actorAABB->GetPos());
		glm::vec3 totalOverlapVolume	= (actorAABB->GetExtents() + otherAABB->GetExtents()) / 2.f;		// Use combined half-extents to get AABB 'radii'
		glm::vec3 overlapVec			= totalOverlapVolume - betweenVec;									

		// Get collision normal by basing it off the smallest axis of overlap
		float smallestAxis		= std::min((std::min(overlapVec.x, overlapVec.y)), overlapVec.z);
		glm::vec3 collNormal	= glm::vec3();

		if (smallestAxis == overlapVec.x) {
			collNormal = overlapVec.x < 0 ? glm::vec3(-1, 0, 0) : glm::vec3(1, 0, 0);
		}
		if (smallestAxis == overlapVec.y) {
			collNormal = overlapVec.y < 0 ? glm::vec3(0, -1, 0) : glm::vec3(0, 1, 0);
		}
		if (smallestAxis == overlapVec.z) {
			collNormal = overlapVec.z < 0 ? glm::vec3(0, 0, -1) : glm::vec3(0, 0, 1);
		}

		a_collision.collisionNormal = collNormal;
		a_collision.overlap			= smallestAxis;															// Base off the smallest overlap axis

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
*	@brief From a list of objects, check every object against every other object and record collisions for this frame.
*	@param a_objects is the list of objects to check collisions in.
*	@return void.
*/
void Scene::DetectCollisions(const std::vector<Rigidbody*>& a_objects)
{
	// Loop through all objects and check all actor objects against all other objects
	for (auto actor_iter = a_objects.begin(); actor_iter != a_objects.end(); ++actor_iter) {
		
		// Second for loop is shifted one to the right in order to access the 'others'
		for (auto other_iter = actor_iter + 1; other_iter != a_objects.end(); ++other_iter) {
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

#if 1
#pragma region Octal Space Partitioning: Separate Plane Collision Checks
	if (b_partitionCollisions && m_objects.size() > 1) {			// No point checking collisions if only one object in the scene
		// Find and store pointers to all planes in the scene
		std::vector<Rigidbody*> planes;

		for (auto obj : m_objects) {
			if (obj->GetShape() == PLANE) {
				planes.push_back(obj);
			}
		}

		// Check every plane against every other non-plane object
		for (auto plane : planes) {

			for (auto obj : m_objects) {
				// Do not check object if it is a plane or apart of the current volume (avoids duplicate collision detections)
				if (obj->GetShape() == PLANE || std::find(a_objects.begin(), a_objects.end(), obj) == a_objects.end()) {

					continue;
				}

				Collision tempPlaneCollision(plane, obj);

				if (obj->GetShape() == SPHERE) {

					if (IsColliding_Plane_Sphere(tempPlaneCollision)) {

						m_collisions.push_back(tempPlaneCollision);
					};
				}
				if (obj->GetShape() == AA_BOX) {

					if (IsColliding_Plane_AABB(tempPlaneCollision)) {

						m_collisions.push_back(tempPlaneCollision);
					}
				}
			}
		}

	}
#pragma endregion
#endif
}

/**
*	@brief Apply impulse force to colliding objects to knock them back.
*	@return void.
*/
void Scene::ResolveCollisions() {
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
				coll.actor->SetPos(coll.actor->GetPos() + (-overlapAccountVec));
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
	// Get average restitution of dynamic objects
	float restitution = (a_collision.actor->GetRestitution() + a_collision.other->GetRestitution()) / 2.f;

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

	// Get restitution of dynamic object
	float restitution = dynamicObj->GetRestitution();

	// Use collision resolution equation to find impulse knockback force.
	// NOTE: Because one object is static, assume it has infinite mass and that the relative velocity is the velocity of the dynamic object
	float impulseScale =
		glm::dot(-(1 + restitution) * dynamicObj->GetVel(), a_collision.collisionNormal) /
		(1 / dynamicObj->GetMass());

	// Apply impulse resolution force along collision normal for dynamic object
	dynamicObj->ApplyImpulseForce(impulseScale * a_collision.collisionNormal);
}
