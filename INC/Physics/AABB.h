#pragma once

#include "Physics\Rigidbody.h"
#include <glm/vec2.hpp>
#include <vector>

namespace Physebs {
	class AABB : public Rigidbody {
	public:
		AABB(const glm::vec3& a_extents = DEFAULT_AABB,
			const glm::vec3& a_pos = glm::vec3(), float a_mass = DEFAULT_MASS, float a_frict = DEFAULT_FRICTION,
			bool a_dynamic = true, const glm::vec4& a_color = DEFAULT_COLOR		// Make AABBs static by default
		);

		virtual ~AABB();

		virtual void Draw();

		float*				GetExtentsRef()							{ return &m_extents.x; }
		const glm::vec3&	GetExtents() const						{ return m_extents; }
		void				SetExtents(const glm::vec3& a_extents)	{ m_extents = a_extents; }

		/**
		*	@brief Check if point is within confines of volume (defined by min and max)
		*	@param a_pt is the point to check.
		*	@param a_min is the minimum point of the volume.
		*	@param a_max is the maximum point of the volume.
		*	@return TRUE: Point is inside volume | FALSE: Point is outside volume
		*/
		static bool PointInMinMax(const float a_pt[3], const float a_min[3], const float a_max[3]) {
			return (a_pt[0] > a_min[0] && a_pt[0] < a_max[0] &&
				a_pt[1] > a_min[1] && a_pt[1] < a_max[1] &&
				a_pt[2] > a_min[2] && a_pt[2] < a_max[2]);
		}

		// NOTE: Do not return references because functions are returning temporary memory
		glm::vec3	CalculateMin() const;
		glm::vec3	CalculateMax() const;
		std::vector<glm::vec3> CalculateCorners() const;
	protected:
		glm::vec3 m_extents;
	private:
	};
}