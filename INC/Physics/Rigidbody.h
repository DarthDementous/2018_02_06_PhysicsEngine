#pragma once

#include <glm/vec3.hpp>
#include <glm/vec4.hpp>
#include "PhysebsUtility_Literals.h"

/**
*	@brief Pure virtual class for objects affected by physics.
*/
namespace Physebs {
	
	enum eShape {SPHERE, PLANE, AA_BOX};			// enums can't have name of an existing data-type or compiler will get confused (e.g. AABB)

	class Rigidbody {
	public:
		// Constructor and destructor can be public because a variable of type Rigidbody can't exist if its pure virtual
		Rigidbody(
			const glm::vec3& a_pos = glm::vec3(), float a_mass = DEFAULT_MASS, float a_frict = DEFAULT_FRICTION, 
			bool a_dynamic = true, const glm::vec4& a_color = DEFAULT_COLOR			// Alpha of 1 by default
		);

		virtual ~Rigidbody() = 0;	// Inherited classes must define a destructor

		virtual void ApplyForce(const glm::vec3& a_force);
		virtual void ApplyImpulseForce(const glm::vec3& a_force);

		virtual void Update(float a_dt);
		virtual void Draw() = 0;

		float*				GetPosRef()							{ return &m_pos.x; }
		const glm::vec3&	GetPos() const						{ return m_pos; }
		void				SetPos(const glm::vec3& a_pos)		{ m_pos = a_pos; }

		const glm::vec3&	GetVel() const						{ return m_vel; }
		void				SetVel(const glm::vec3& a_vel)		{ m_vel = a_vel; }
		
		const glm::vec3&	GetAccel() const					{ return m_accel; }
		void				SetAccel(const glm::vec3& a_accel)	{ m_accel = a_accel; }

		float*				GetColorRef()						{ return &m_color.r; }
		const glm::vec4&	GetColor() const					{ return m_color; }
		void				SetColor(const glm::vec4& a_color)	{ m_color = a_color; }

		float*				GetMassRef()						{ return &m_mass; }
		float				GetMass() const						{ return m_mass; }
		void				SetMass(float a_mass)				{ m_mass = a_mass; }

		float*				GetFrictRef()						{ return &m_frict; }
		float				GetFrict() const					{ return m_frict; }
		void				SetFrict(float a_frict)				{ m_frict = a_frict; }

		eShape				GetShape() const					{ return m_shape; }

		bool*				GetIsDynamicRef()					{ return &b_dynamic; }
		bool				GetIsDynamic() const				{ return b_dynamic; }
		void				SetIsDynamic(bool a_isDynamic)		{ b_dynamic = a_isDynamic; }
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
