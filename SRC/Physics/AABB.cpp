#include "Physics/AABB.h"
#include <Gizmos.h>
#include <glm/ext.hpp>

using namespace Physebs;

AABB::AABB(
	const glm::vec3 & a_extents,
	const glm::vec3 & a_pos, float a_mass, float a_frict, bool a_dynamic, const glm::vec4 & a_color
) :
	m_extents(a_extents),
	Rigidbody(a_pos, a_mass, a_frict, a_dynamic, a_color) // Call base constructor to handle assigning inherited variables
{
	m_shape = AA_BOX;
}

AABB::~AABB()
{
}

void AABB::Draw()
{
	// NOTE: Bootstrap treats extents like half-extents when drawing AABBs despite the parameter name so need to halve extents
	aie::Gizmos::addAABBFilled(m_pos, m_extents / 2.f, m_color);	
}

/**
*	@brief Calculate and return bottom-left most point on the 3D AABB from current half extents and position
*	@return Minimum point of the AABB.
*/
const glm::vec3 & AABB::GetMin() const
{
	glm::vec3 halfExtents = m_extents / 2.f;

	return m_pos - halfExtents;
}

/**
*	@brief Calculate and return top-right most point on the 3D AABB from current half extents and position
*	@return Maximum point of the AABB.
*/
const glm::vec3 & AABB::GetMax() const
{
	glm::vec3 halfExtents = m_extents / 2.f;

	return m_pos + halfExtents;
}
