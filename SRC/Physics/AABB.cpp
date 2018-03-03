#include "Physics/AABB.h"
#include <Gizmos.h>
#include <glm/ext.hpp>

using namespace Physebs;

AABB::AABB(
	const glm::vec3 & a_extents,
	const glm::vec3 & a_pos, float a_mass, float a_frict, bool a_dynamic, const glm::vec4 & a_color, float a_restitution
) :
	m_extents(a_extents),
	Rigidbody(a_pos, a_mass, a_frict, a_dynamic, a_color, a_restitution) // Call base constructor to handle assigning inherited variables
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
glm::vec3 AABB::CalculateMin() const
{
	glm::vec3 halfExtents = m_extents / 2.f;

	return m_pos - halfExtents;
}

/**
*	@brief Calculate and return top-right most point on the 3D AABB from current half extents and position
*	@return Maximum point of the AABB.
*/
glm::vec3 AABB::CalculateMax() const
{
	glm::vec3 halfExtents = m_extents / 2.f;

	return m_pos + halfExtents;
}

/**
*	@brief Using half extents and position, calculate vector of 8 corner positions.
*	@return Vector of AABB corners (vec3s)
*/
std::vector<glm::vec3> AABB::CalculateCorners() const
{
	glm::vec3 halfExtents = m_extents / 2.f;
	std::vector<glm::vec3> corners;

	// Back top left corner
	corners.push_back(glm::vec3(m_pos.x - halfExtents.x, m_pos.y + halfExtents.y, m_pos.z - halfExtents.z));

	// Back top right corner
	corners.push_back(glm::vec3(m_pos.x + halfExtents.x, m_pos.y + halfExtents.y, m_pos.z - halfExtents.z));

	// Forward top left corner
	corners.push_back(glm::vec3(m_pos.x - halfExtents.x, m_pos.y + halfExtents.y, m_pos.z + halfExtents.z));

	// Forward top right corner
	corners.push_back(glm::vec3(m_pos.x + halfExtents.x, m_pos.y + halfExtents.y, m_pos.z + halfExtents.z));

	// Back bottom left corner
	corners.push_back(glm::vec3(m_pos.x - halfExtents.x, m_pos.y - halfExtents.y, m_pos.z - halfExtents.z));

	// Back bottom right corner
	corners.push_back(glm::vec3(m_pos.x + halfExtents.x, m_pos.y - halfExtents.y, m_pos.z - halfExtents.z));

	// Forward bottom left corner
	corners.push_back(glm::vec3(m_pos.x - halfExtents.x, m_pos.y - halfExtents.y, m_pos.z + halfExtents.z));

	// Forward bottom right corner
	corners.push_back(glm::vec3(m_pos.x + halfExtents.x, m_pos.y - halfExtents.y, m_pos.z + halfExtents.z));

	return corners;
}
