#pragma once

#include "Physics/Constraint.h"

namespace Physebs {
	class Spring : public Constraint {
	public:
		Spring(Rigidbody* a_attachedActor, Rigidbody* a_attachedOther, const glm::vec4& a_color = DEFAULT_CONSTRAINT_COLOR,
			float a_springiness = DEFAULT_SPRINGINESS, float a_restLength = DEFAULT_SPRING_LENGTH);
		~Spring();

		virtual void Constrain();
		virtual void Draw();

		float* GetSpringinessRef()	{ return &m_springiness; }
		float *GetRestLengthRef()	{ return &m_restLength; }
	protected:
		float m_springiness;	// Scale of force to apply to attached rigidbodies
		float m_restLength;		// Distance attached rigidbodies must be at before being constrained
	private:
	};
}
