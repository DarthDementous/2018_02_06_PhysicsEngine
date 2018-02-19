#include "Physics/Plane.h"
#include <Gizmos.h>
#include <glm/ext.hpp>

using namespace Physebs;

Plane::Plane(
	const glm::vec3 & a_normal,
	const glm::vec3 & a_pos, float a_mass, float a_frict, bool a_dynamic, const glm::vec4 & a_color
) :
	m_normal(a_normal),
	Rigidbody(a_pos, a_mass, a_frict, a_dynamic, a_color) // Call base constructor to handle assigning inherited variables
{
	m_shape = PLANE;

	// Ensure the vector passed in for normal is a unit vector (normalised) if its not 0 to avoid nan errors
	if (glm::length(m_normal) != 0) {
		m_normal = glm::normalize(m_normal);
	}
}

Plane::~Plane()
{
}

void Plane::Draw()
{

	//TODO: Draw tri that's as wide as the camera bounds to give the impression of being an infinite plane
	//aie::Gizmos::add2DTri()
}
