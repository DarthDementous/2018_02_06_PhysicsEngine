#pragma once

#include <vector>
#include <glm/vec3.hpp>
#include "PhysebsUtility.h"

namespace Physebs {
	class Rigidbody;
	class Sphere;

	/**
	*	@brief Structure for holding collision data.
	*/
	struct Collision {
		Collision(Rigidbody* a_actor, Rigidbody* a_other) : actor(a_actor), other(a_other) {}

		Rigidbody* actor;
		Rigidbody* other;
	};

	/**
	*	@brief Class that holds onto rigidbody objects and handles their physics (gravity, forces, collisions).
	*	NOTE: Standards for collision are creating a vector from A to B.
	*/
	class Scene {
	public:
		Scene(const glm::vec3& a_gravityForce = glm::vec3(0, DEFAULT_GRAVITY, 0), // Default gravity force pushes down
			const glm::vec3& a_globalForce = glm::vec3());
		~Scene();

		void FixedUpdate(float a_dt);
		void Draw();
		
		void AddObject(Rigidbody* a_obj);
		void RemoveObject(Rigidbody* a_obj);

		void ApplyGlobalForce();

		static bool IsColliding_Sphere_Sphere(Sphere* a_actor, Sphere* a_other);

		const std::vector<Rigidbody*>& GetObjects()	const				{ return m_objects; }

		const glm::vec3&	GetGravity() const							{ return m_gravity; }
		void				SetGravity(const glm::vec3& a_gravity)		{ m_gravity = a_gravity; }

		const glm::vec3&	GetGlobalForce() const						{ return m_globalForce; }
		void				SetGlobalForce(const glm::vec3& a_force)	{ m_globalForce = a_force; }
	protected:
		glm::vec3 m_gravity;
		glm::vec3 m_globalForce;				// Force that will affect all objects

		std::vector<Rigidbody*> m_objects;
		std::vector<Collision>	m_collisions;	// Hold onto all collisions that have occured in the frame for collision resolution

		// Fixed Update variables
		float m_fixedTimeStep;				// The fixed time between updates of the scene
		float m_accumulatedTime;			// How much time overflow there is between fixed updates
	private:
		void Update();						// Update functionality with fixed time step
		void ApplyGravity();				// Only want scene to be able to apply gravity to keep consistency
		void DetectCollisions();			// Object collisions are only handled within the scene
		void ResolveCollisions();			// Apply appropriate forces to objects that have collided
	};
}