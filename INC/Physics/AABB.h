#pragma once

#include "Physics\Rigidbody.h"
#include <glm/vec2.hpp>

namespace Physebs {
	class AABB : public Rigidbody {
	public:
		AABB(const glm::vec3& a_extents = DEFAULT_AABB,
			const glm::vec3& a_pos = glm::vec3(), float a_mass = DEFAULT_MASS, float a_frict = DEFAULT_FRICTION,
			bool a_dynamic = false, const glm::vec4& a_color = DEFAULT_COLOR		// Make AABBs static by default
		);

		virtual ~AABB();

		virtual void Draw();

		float*				GetExtentsRef()							{ return &m_extents.x; }
		const glm::vec3&	GetExtents() const						{ return m_extents; }
		void				SetExtents(const glm::vec3& a_extents)	{ m_extents = a_extents; }
	protected:
		glm::vec3 m_extents;
	private:
	};
}