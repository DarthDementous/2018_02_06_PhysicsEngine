#pragma once

#include <glm/vec3.hpp>
#include "PhysebsUtility.h"

/**
*	@brief Library for custom 3D physics engine.
*/
namespace Physebs {
	
	class Rigidbody {
	public:
		Rigidbody(const glm::vec3& a_pos = glm::vec3(), float a_mass = DEFAULT_MASS, float a_frict = DEFAULT_FRICTION);
		virtual ~Rigidbody();

		virtual void ApplyForce(const glm::vec3& a_force);

		virtual void Update(float a_dt);
		virtual void Draw() {}

		const glm::vec3&	GetPos() const						{ return m_pos; }
		void				SetPos(const glm::vec3& a_pos)		{ m_pos = a_pos; }

		const glm::vec3&	GetVel() const						{ return m_vel; }
		void				SetVel(const glm::vec3& a_vel)		{ m_vel = a_vel; }
		
		const glm::vec3&	GetAccel() const					{ return m_accel; }
		void				SetAccel(const glm::vec3& a_accel)	{ m_accel = a_accel; }

		const float			GetMass() const						{ return m_mass; }
		void				SetMass(float a_mass)				{ m_mass = a_mass; }
	protected:
		glm::vec3 m_pos;
		glm::vec3 m_vel;
		glm::vec3 m_accel;	

		float m_mass;
		float m_frict;

	private:
	};
}
