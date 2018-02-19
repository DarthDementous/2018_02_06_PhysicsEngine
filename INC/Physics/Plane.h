#pragma once

#include "Physics\Rigidbody.h"
#include <glm/vec2.hpp>

namespace Physebs {
	class Plane : public Rigidbody {
	public:
		Plane(const glm::vec3& a_normal = DEFAULT_PLANE_NORMAL,
			const glm::vec3& a_pos = glm::vec3(), float a_mass = DEFAULT_MASS, float a_frict = DEFAULT_FRICTION,
			bool a_dynamic = false, const glm::vec4& a_color = glm::vec4()		// Make planes static by default
		);

		virtual ~Plane();

		virtual void Draw();

		const glm::vec3&	GetNormal() const { return m_normal; }
		void				SetNormal(const glm::vec3& a_normal) { m_normal = a_normal; }
	protected:
		glm::vec3 m_normal;
	private:
	};
}