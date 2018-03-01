#pragma once

#include <vector>
#include <glm/vec3.hpp>
#include <glm/vec4.hpp>
#include "PhysebsUtility.h"
#include "Octree\Octree.h"
#include <Gizmos.h>
#include <iostream>

namespace brandonpelfrey {
	class Octree;
}

namespace Physebs {
	class Rigidbody;
	class Sphere;
	class Plane;

	class Constraint;

	/**
	*	@brief Structure for holding collision data.
	*/
	struct Collision {
		Collision(Rigidbody* a_actor, Rigidbody* a_other, const glm::vec3& a_collisionNormal = glm::vec3(), float a_overlap = 0.f) 
			: actor(a_actor), other(a_other), collisionNormal(a_collisionNormal), overlap(a_overlap) {}

	public:
		Rigidbody* actor;
		Rigidbody* other;

		float overlap;				// The overlap between the colliding object's shapes

		glm::vec3 collisionNormal;	// The direction to base collision knockback on

		/**
		*	@brief Swap actor with other to use the main is colliding function in order to avoid duplicate code.
		*	@return void.
		*/
		void SwapObjects() {
			Rigidbody * tmp = actor;

			actor = other;
			other = tmp;
		}
	};

	/**
	*	@brief Class that holds onto rigidbody objects and handles their physics (gravity, forces, collisions).
	*	NOTE: Standards for collision are creating a vector from A to B.
	*/
	class Scene {
	public:
		Scene(const glm::vec3& a_gravityForce = glm::vec3(0, DEFAULT_GRAVITY, 0), const glm::vec3& a_globalForce = glm::vec3(),	 // Default gravity force pushes down
			const glm::vec3& a_simulationOrigin = glm::vec3(), const glm::vec3& a_simulationHalfExtents = DEFAULT_SIMULATION_HALFEXTENTS);
		~Scene();

		void FixedUpdate(float a_dt);
		void Draw();
		
		void AddObject(Rigidbody* a_obj);
		void RemoveObject(Rigidbody* a_obj);

		void AddConstraint(Constraint* a_constraint);
		void RemoveConstraint(Constraint* a_constraint);

		void ApplyGlobalForce();

		void PartitionCollisions();

		static bool IsColliding_Sphere_Sphere(Collision& a_collision);
		static bool IsColliding_Sphere_Plane(Collision& a_collision);
		static bool IsColliding_Sphere_AABB(Collision& a_collision);

		static bool IsColliding_Plane_Sphere(Collision& a_collision);
		static bool IsColliding_Plane_AABB(Collision& a_collision);
		
		static bool IsColliding_AABB_Plane(Collision& a_collision);
		static bool IsColliding_AABB_Sphere(Collision& a_collision);
		static bool IsColliding_AABB_AABB(Collision& a_collision);

		const std::vector<Rigidbody*>& GetObjects()	const				{ return m_objects; }
		const std::vector<Constraint*>& GetConstraints() const			{ return m_constraints; }

		const glm::vec3&	GetGravity() const							{ return m_gravity; }
		void				SetGravity(const glm::vec3& a_gravity)		{ m_gravity = a_gravity; }

		const glm::vec3&	GetGlobalForce() const						{ return m_globalForce; }
		void				SetGlobalForce(const glm::vec3& a_force)	{ m_globalForce = a_force; }
	protected:
		glm::vec3 m_gravity;
		glm::vec3 m_globalForce;				// Force that will affect all objects

		std::vector<Rigidbody*>		m_objects;
		std::vector<Constraint*>	m_constraints;	// Hold onto all constraints between objects
		std::vector<Collision>		m_collisions;	// Hold onto all collisions that have occured in the frame for collision resolution

		// Optimization
		//brandonpelfrey::Octree*		m_spatialPartitionTree = nullptr;	// Simple octree library implementation for segmenting collision detections into AABBs
	protected:
		glm::vec3						m_simulationOrigin;							// Origin point of initial collision detection AABB
		glm::vec3						m_simulationHalfExtents;					// Half extents of initial collision detection AABB  

		// Fixed Update variables
		float m_fixedTimeStep;														// The fixed time between updates of the scene
		float m_accumulatedTime;													// How much time overflow there is between fixed updates
	private:
		void Update();																// Update functionality with fixed time step
		void ApplyGravity();														// Only want scene to be able to apply gravity to keep consistency
		void DetectCollisions(const std::vector<Rigidbody*>& a_objects);			// Object collisions are only handled within the scene
		void ResolveCollisions();													// Apply appropriate forces to objects that have collided
		void ApplyKnockback_Dynamic(Collision& a_collision);
		void ApplyKnockback_Static(Collision& a_collision);

#pragma region Spatial Partitioning
		/**
		*	@brief Structure for holding onto objects in a partition volume within an octree.
		*/
		struct PartitionNode {

			PartitionNode() {
				
				// Generate random color for volume
				//debugColor = glm::vec4((rand() % 255) / 255.f, (rand() % 255) / 255.f, (rand() % 255) / 255.f, 1.f);	 // Generate random numbers between 0-255 and then divide by 255 to get RGB float values
				debugColor = glm::vec4(1, 0, 0, 0.25f);
			}

			Scene*					scene = nullptr;						// What scene contained objects are apart of
			glm::vec4				debugColor = glm::vec4();				// What color to draw contained objects in. All values set to 0 by default.
			std::vector<Rigidbody*> containedObjects;						// List of pointers to object that are inside the partition volume
		};

		Octree<PartitionNode>*	m_spatialPartitionTree = nullptr;

		/**
		*	@brief Inherited callback class used for detecting collisions with contained objects in a partition volume in an octree.
		*/
		class OctreeCallbackDetectCollisions : public Octree<PartitionNode>::Callback {

			virtual bool operator()(const float min[3], const float max[3], PartitionNode& nodeData) {
				// Call detect collisions in the scene with the contained objects in the volume
				nodeData.scene->DetectCollisions(nodeData.containedObjects);

				// Continue to detect collisions in the rest of the octree volumes
				return true;
			}
		};

		/**
		*	@brief Inherited callback class used for drawing an AABB for each partition volume in octree.
		**/
		class OctreeCallbackDebug : public Octree<PartitionNode>::Callback {

		public:
			// Override callback operator called for every octree node. NOTE: If function returns false, traversal will be broken early
			virtual bool operator()(const float min[3], const float max[3], PartitionNode& nodeData) {
				// Calculate extents and pos to draw AABB representative of current octree volume
				glm::vec3 currentMin = glm::vec3(min[0], min[1], min[2]);
				glm::vec3 currentMax = glm::vec3(max[0], max[1], max[2]);

				glm::vec3 pos = (currentMin + currentMax) / 2.f;
				glm::vec3 halfExtents = (currentMax - currentMin) / 2.f;

				aie::Gizmos::addAABB(pos, halfExtents, nodeData.debugColor);

				// Make AABBs for the rest of the octree volumes
				return true;
			}
		};
#pragma endregion

	
	};
}