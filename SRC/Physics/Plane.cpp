#include "Physics/Plane.h"
#include <Gizmos.h>
#include <glm/ext.hpp>

using namespace Physebs;

Plane::Plane(
	const glm::vec3 & a_normal, float a_originDist,
	const glm::vec3 & a_pos, float a_mass, float a_frict, bool a_dynamic, const glm::vec4 & a_color
) :
	m_normal(a_normal), m_originDist(a_originDist),
	Rigidbody(a_pos, a_mass, a_frict, a_dynamic, a_color) // Call base constructor to handle assigning inherited variables
{
	m_shape = PLANE;

	// Ensure the vector passed in for normal is a unit vector (length of 1) if its not 0 to avoid nan errors
	if (glm::length(m_normal) != 0) {
		m_normal = glm::normalize(m_normal);
	}
}

Plane::~Plane()
{
}

void Plane::Draw()
{
	// 1. Find center position of plane using normal and distance
	glm::vec3 planePos = m_normal * m_originDist;
	// 2. Find direction of the plane lines (what is physically drawn) by finding vector perpendicular to the normal (cross-product with arbitrary vector)
	glm::vec3 arbitraryVec	= m_normal + 1.f;		// Slightly off-centered vector ensures its never parallel (cross-product returns 0, 0, 0);
	glm::vec3 planeLineDir	= glm::normalize(glm::cross(m_normal, arbitraryVec));
	glm::vec3 planeLineDir2 = glm::normalize(glm::cross(planeLineDir, m_normal));
	// 3. Calculate four points along plane lines required to draw plane by using max draw distance to simulate it being infinite
	glm::vec3 v1 = planePos + (planeLineDir * PLANE_DRAW);
	glm::vec3 v2 = planePos - (planeLineDir * PLANE_DRAW);
	glm::vec3 v3 = planePos + (planeLineDir2 * PLANE_DRAW);
	glm::vec3 v4 = planePos - (planeLineDir2 * PLANE_DRAW);

	aie::Gizmos::addTri(v1, v2, v3, m_color);
	aie::Gizmos::addTri(v4, v2, v1, m_color);
}

void Plane::Update(float a_dt)
{
	// Static rigidbodies do not move
	if (b_dynamic) {
		// Apply dampening (negative velocity scaled by friction)
		ApplyForce(-m_vel * m_frict);

		// Calculate velocity
		m_vel += m_accel * a_dt;

		// Truncate velocity with an epsilon
		if (glm::length(m_vel) < EPSILON) {
			// Zero out velocity to avoid floating point errors
			m_vel = glm::vec3();
		}

		/// Calculate distance from origin
		// Dot-product velocity vector with normal to determine whether velocity its positive or negative
		float	velDotCheck = glm::dot(m_normal, m_vel * a_dt);
		bool	b_negVel	= (velDotCheck < 0) ? true : false;
		float	velLength	= glm::length(velDotCheck);

		// Convert velocity into single float and negate if velocity is negative to plane normal
		m_originDist += (b_negVel) ? -velLength : velLength;

		/// Calculate position
		m_pos = m_normal * m_originDist;
	}

	// Reset acceleration so it gets re-calculated (regardless of static or dynamic)
	m_accel = glm::vec3();
}
