#pragma once

#include "Physics/Constraint.h"

namespace Physebs {
	class Spring : public Constraint {
	public:
		Spring(Rigidbody* a_attachedActor, Rigidbody* a_attachedOther, const glm::vec4& a_color = DEFAULT_CONSTRAINT_COLOR,
			float a_springiness = DEFAULT_SPRINGINESS, float a_restLength = DEFAULT_SPRING_LENGTH, float a_springDampening = DEFAULT_FRICTION);
		~Spring();

		virtual void Constrain();
		virtual void Draw();

		float* GetSpringinessRef()	{ return &m_springiness; }
		float* GetRestLengthRef()	{ return &m_restLength; }
		float* GetDampeningRef()	{ return &m_springDampening; }
	protected:
		float m_springiness;		// Scale of force to apply to attached rigidbodies
		float m_restLength;			// Distance attached rigidbodies must be at before being constrained
		float m_springDampening;	// How quickly energy from spring dissipates
	private:
	};
}
