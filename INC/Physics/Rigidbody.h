#pragma once

#include <glm/vec3.hpp>
#include <glm/vec4.hpp>
#include "PhysebsUtility.h"

/**
*	@brief Pure virtual class for objects affected by physics.
*/
namespace Physebs {
	
	enum eShape {SPHERE, PLANE, AABB};

	class Rigidbody {
	public:
		// Constructor and destructor can be public because a variable of type Rigidbody can't exist if its pure virtual
		Rigidbody(
			const glm::vec3& a_pos = glm::vec3(), float a_mass = DEFAULT_MASS, float a_frict = DEFAULT_FRICTION, 
			bool a_dynamic = true, const glm::vec4& a_color = glm::vec4()
		);

		virtual ~Rigidbody() = 0;	// Inherited classes must define a destructor

		virtual void ApplyForce(const glm::vec3& a_force);
		virtual void ApplyImpulseForce(const glm::vec3& a_force);

		virtual void Update(float a_dt);
		virtual void Draw() = 0;

		const glm::vec3&	GetPos() const						{ return m_pos; }
		void				SetPos(const glm::vec3& a_pos)		{ m_pos = a_pos; }

		const glm::vec3&	GetVel() const						{ return m_vel; }
		void				SetVel(const glm::vec3& a_vel)		{ m_vel = a_vel; }
		
		const glm::vec3&	GetAccel() const					{ return m_accel; }
		void				SetAccel(const glm::vec3& a_accel)	{ m_accel = a_accel; }

		const float			GetMass() const						{ return m_mass; }
		void				SetMass(float a_mass)				{ m_mass = a_mass; }

		const eShape		GetShape() const					{ return m_shape; }
	protected:
		glm::vec3 m_pos;
		glm::vec3 m_vel;
		glm::vec3 m_accel;	

		glm::vec4 m_color;

		float m_mass;
		float m_frict;

		bool b_dynamic;			// Whether rigidbody is affected by physics or not

		eShape m_shape;			// Be able to quickly determine what shape we're dealing with. DO NOT ALLOW TO BE MODIFIED.
	private:
	};
}
