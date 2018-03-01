#pragma once

#include "Physics\Rigidbody.h"
#include <glm/vec2.hpp>

namespace Physebs {
	class Plane : public Rigidbody {
	public:
		Plane(const glm::vec3& a_normal = DEFAULT_PLANE_NORMAL, float a_originDist = 0,
			const glm::vec3& a_pos = glm::vec3(), float a_mass = DEFAULT_MASS, float a_frict = DEFAULT_FRICTION,
			bool a_dynamic = false, const glm::vec4& a_color = DEFAULT_COLOR		// Make planes static by default
		);

		virtual ~Plane();

		virtual void Draw();

		// Override update to ensure acceleration is applied to distance from origin instead of position
		virtual void Update(float a_dt);

		float*				GetDistRef()							{ return &m_originDist; }
		float				GetDist() const							{ return m_originDist; }
		void				SetDist(float a_dist)					{ m_originDist = a_dist; }

		float*				GetNormalRef()							{ return &m_normal.x; }
		const glm::vec3&	GetNormal() const						{ return m_normal; }
		void				SetNormal(const glm::vec3& a_normal)	{ m_normal = a_normal; }
	protected:
		glm::vec3 m_normal;		// The direction the plane is facing in

		float m_originDist;		// How far away from the origin the plane is (+ if in positive range, - if in negative range)
	private:
	};
}