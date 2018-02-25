#include "Physics/Spring.h"
#include "Physics/Rigidbody.h"
#include <Gizmos.h>
#include <glm/ext.hpp>
#include <glm/vec3.hpp>

using namespace Physebs;

Spring::Spring
(
	Rigidbody* a_attachedActor, Rigidbody* a_attachedOther,
	float a_springiness, float a_restLength
) :
	Constraint(a_attachedActor, a_attachedOther),
	m_springiness(a_springiness), m_restLength(a_restLength)
{
}

Spring::~Spring()
{
}

void Spring::Constrain()
{
	// 1. Get difference between target length and current distance between attached rigidbodies
	glm::vec3	springVec = m_attachedOther->GetPos() - m_attachedActor->GetPos();
	float		currentDisplacement = m_restLength - glm::length(springVec);

	// 2. Calculate relative velocity depending on type of Rigidbody actor and other are
	glm::vec3 relativeVel = glm::vec3();
	
	/// Dynamic vs dynamic
	if (m_attachedActor->GetIsDynamic() && m_attachedOther->GetIsDynamic()) {
		// B - A for consistency with spring vector
		relativeVel = m_attachedOther->GetVel() - m_attachedActor->GetVel();
	}

	/// Dynamic vs static
	if (m_attachedActor->GetIsDynamic() && !m_attachedOther->GetIsDynamic()) {
		// Other has no velocity so relative velocity is actor's
		relativeVel = m_attachedActor->GetVel();
	}

	/// Static vs dynamic
	if (!m_attachedActor->GetIsDynamic() && m_attachedOther->GetIsDynamic()) {
		// Actor has no velocity so relative velocity is other's
		relativeVel = m_attachedOther->GetVel();
	}

	// 3. Determine the scale of how much objects should be sprung back from springiness and disparity between current length and target length
	float retractScale			= m_springiness * currentDisplacement;

	// 4. Calculate final force to impart on attached Rigidbodies this function call with Hooke's law
	// NOTE: If constrain condition is not met (not at resting length) then only dampening will be applied.
	glm::vec3 finalSpringForce	= springVec * retractScale;		// Apply force along spring

	m_attachedActor->ApplyForce(-finalSpringForce);
	m_attachedOther->ApplyForce(finalSpringForce);

}

void Spring::Draw()
{
	// Draw line between attached Rigidbodies to simulate the 'spring'
	aie::Gizmos::addLine(m_attachedActor->GetPos(), m_attachedOther->GetPos(), glm::vec4(1, 1, 0, 1));
}
