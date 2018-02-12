#include "Physics\Rigidbody.h"

using namespace Physebs;



Rigidbody::Rigidbody(const glm::vec3 & a_pos, float a_mass, float a_frict) : m_pos(a_pos), m_mass(a_mass), m_frict(a_frict)
{
	// Initialise to null vec3 (0, 0, 0)
	m_vel	= glm::vec3();
	m_accel = glm::vec3();
}

Rigidbody::~Rigidbody()
{
}

/**
*	@brief Modify acceleration with a given force
*	@param a_force is the vector 3 force being applied
*	@return void.
*/
void Rigidbody::ApplyForce(const glm::vec3& a_force)
{
	// f = m * a | a = f / m
	m_accel += a_force / m_mass;
}

void Rigidbody::Update(float a_dt)
{
	// Apply friction (negative velocity scaled by friction)
	ApplyForce(-m_vel * m_frict);

	// Calculate velocity
	m_vel += m_accel * a_dt;

	// Calculate position
	m_pos += m_vel * a_dt;

	// Reset acceleration so it gets re-calculated
	m_accel = glm::vec3();
}
