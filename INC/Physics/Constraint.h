#pragma once

#include <glm/vec4.hpp>
#include "PhysebsUtility_Literals.h"

namespace Physebs {
	enum eConstraint { SPRING, JOINT };

	class Rigidbody;

	/**
	*	@brief Pure abstract class representing a relationship between two objects that affects how they exist in relation to other in the simulation.
	*/
	class Constraint {
	public:
		Constraint(Rigidbody* a_attachedActor, Rigidbody* a_attachedOther, const glm::vec4& a_color = DEFAULT_CONSTRAINT_COLOR);
		virtual ~Constraint();

		virtual void Update();
		virtual void Constrain() = 0;

		virtual void Draw() = 0;

		bool ContainsObj(Rigidbody* a_obj);

		eConstraint GetType() const { return m_type; }

		const glm::vec4&	GetColor()		{ return m_color; }
		float*				GetColorRef()	{ return &m_color.r; }

		Rigidbody*	GetAttachedActor() const { return m_attachedActor; }
		Rigidbody*	GetAttachedOther() const { return m_attachedOther; }
	protected:
		Rigidbody* m_attachedActor;
		Rigidbody* m_attachedOther;

		eConstraint m_type;

		glm::vec4	m_color;
	private:
	};
}
