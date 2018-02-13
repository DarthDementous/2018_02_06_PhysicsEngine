#pragma once

#include "Physics\Rigidbody.h"
#include <glm/vec2.hpp>

namespace Physebs {
	class Sphere : public Rigidbody {
	public:
		Sphere(
			float a_radius = DEFAULT_MASS, const glm::vec2& a_dimensions = DEFAULT_SPHERE,
			const glm::vec3& a_pos = glm::vec3(), float a_mass = DEFAULT_MASS, float a_frict = DEFAULT_FRICTION, 
			bool a_dynamic = true, const glm::vec4& a_color = glm::vec4()
		);
		
		virtual ~Sphere();

		virtual void Draw();

		const float GetRadius() const				{ return m_radius; }
		void		SetRadius(const float a_radius)	{ m_radius = a_radius; }
	protected:
		float m_radius;
		
		glm::ivec2 m_dimensions;			// Level of detail of the drawn sphere in rows and columns
	private:
	};
}