#include "Physics/Plane.h"
#include <Gizmos.h>

using namespace Physebs;

Plane::Plane(
	const glm::vec3 & a_normal,
	const glm::vec3 & a_pos, float a_mass, float a_frict, bool a_dynamic, const glm::vec4 & a_color
) :
	m_normal(a_normal),
	Rigidbody(a_pos, a_mass, a_frict, a_dynamic, a_color) // Call base constructor to handle assigning inherited variables
{
	m_shape = PLANE;
}

Plane::~Plane()
{
}

void Plane::Draw()
{

	//TODO: Draw tri that's as wide as the camera bounds to give the impression of being an infinite plane
	//aie::Gizmos::add2DTri()
}
