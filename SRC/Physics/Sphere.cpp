#include "Physics\Sphere.h"
#include <Gizmos.h>
#include <glm/vec4.hpp>

using namespace Physebs;

Sphere::Sphere(
	float a_radius, const glm::vec2& a_dimensions, 
	const glm::vec3 & a_pos, float a_mass, float a_frict, 
	bool a_dynamic, const glm::vec4& a_color
) :
	m_radius(a_radius), m_dimensions(a_dimensions), 
	Rigidbody(a_pos, a_mass, a_frict, a_dynamic, a_color)		// Call base constructor to handle assigning inherited variables
{
	m_shape = SPHERE;
}

Sphere::~Sphere()
{
}

void Sphere::Draw()
{
	// Add sphere gizmo for testing
	aie::Gizmos::addSphere(m_pos, m_radius, m_dimensions.x, m_dimensions.y, m_color);
}
