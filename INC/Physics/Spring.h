#pragma once

#include "Physics/Constraint.h"
#include "PhysebsUtility.h"

namespace Physebs {
	class Spring : public Constraint {
	public:
		Spring(Rigidbody* a_attachedActor, Rigidbody* a_attachedOther,
			float a_springiness = DEFAULT_SPRINGINESS, float a_restLength = DEFAULT_SPRING_LENGTH);
		~Spring();

		virtual void Constrain();
		virtual void Draw();
	protected:
		float m_springiness;	// Scale of force to apply to attached rigidbodies
		float m_restLength;		// Distance attached rigidbodies must be at before being constrained
	private:
	};
}
