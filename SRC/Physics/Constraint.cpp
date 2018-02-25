#include "Physics/Constraint.h"

using namespace Physebs;

Constraint::Constraint(Rigidbody* a_attachedActor, Rigidbody* a_attachedOther) : 
	m_attachedActor(a_attachedActor), m_attachedOther(a_attachedOther)
{
}

Constraint::~Constraint()
{
}

void Constraint::Update()
{
	Constrain();
}
