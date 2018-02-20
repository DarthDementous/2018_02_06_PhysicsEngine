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
	aie::Gizmos::addAABBFilled(m_pos, m_extents, m_color);
}
