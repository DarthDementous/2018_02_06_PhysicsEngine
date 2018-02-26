#include "Physics/Constraint.h"

using namespace Physebs;

Constraint::Constraint(Rigidbody* a_attachedActor, Rigidbody* a_attachedOther, const glm::vec4& a_color) : 
	m_attachedActor(a_attachedActor), m_attachedOther(a_attachedOther), m_color(a_color)
{
}

Constraint::~Constraint()
{
}

void Constraint::Update()
{
	// Apply force to objects based on constraint conditions
	Constrain();
}

/**
*	@brief Check if constraint has target object as an attached Rigidbody.
*	@param a_obj is the object to check is apart of the constraint.
*	@return TRUE: Constraint has object as an attached Rigidbody | FALSE: Constraint does not have object as an attached Rigidbody.
*/
bool Constraint::ContainsObj(Rigidbody* a_obj)
{
	if (m_attachedActor == a_obj || m_attachedOther == a_obj) {
		return true;
	}
	return false;
}
