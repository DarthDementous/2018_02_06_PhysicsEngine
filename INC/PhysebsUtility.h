#pragma once

#define B_SHOW_PARTITIONS 1			// Whether or not to show AABBs for each volume in the partition tree
#define B_PARTITION_COLLISIONS 1	// Whether to use octal space partitioning optimisation for collisions
#define B_VOLUME_COLORS 1			// Whether to set object colors to the volume they are contained in

// The limit point before a value will be zeroed
#define EPSILON 0.001f

#define DEFAULT_MASS 2.f
#define DEFAULT_FRICTION 8.f
#define DEFAULT_GRAVITY -9.8f

#define DEFAULT_SPHERE glm::ivec2(16, 16)
#define DEFAULT_AABB glm::vec3(4, 4, 4)
#define DEFAULT_PLANE_NORMAL glm::vec3(0, 1, 0)
#define DEFAULT_SIMULATION_HALFEXTENTS glm::vec3(50, 50, 50)

#define MIN_VOLUME_SIZE { 20, 20, 20}

#define DEFAULT_COLOR glm::vec4(0, 0, 0, 1)
#define DEFAULT_CONSTRAINT_COLOR glm::vec4(1, 1, 0, 1)
#define DEFAULT_SELECTION_COLOR glm::vec4(0, 1, 0, 0.5)
#define DEFAULT_ACTOR_SELECTION_COLOR glm::vec4(1, 0, 0, 0.5)
#define DEFAULT_OTHER_SELECTION_COLOR glm::vec4(0, 0, 1, 0.5)
#define DEFAULT_CONSTRAINT_SELECTION_COLOR glm::vec4(1, 0.7, 0, 0.5)

#define CAMERA_NEAR 0.1f
#define CAMERA_FAR 1000.f

#define PLANE_DRAW CAMERA_FAR

#define DEFAULT_SELECTION_RADIUS 4.f

#define DEFAULT_SELECTION_SPHERE glm::ivec2(6, 6)

#define DEFAULT_SPRINGINESS 20.f
#define DEFAULT_SPRING_LENGTH 10.f
#define DEFAULT_SPRING_DAMPENING 0.5f

namespace Physebs {
	/**
	*	@brief Return the smaller of two values, or the first value if both are equal.
	*	@param a_val1 is the first value.
	*	@param a_val2 is the second value.
	*	@return const reference to smallest value.
	*/
	template<class T>
	const T& Min(const T& a_val1, const T& a_val2) {
		return !(a_val2 < a_val1) ? a_val1 : a_val2;		// Return smaller value or val1 if val1 and val2 are equal
	}

	/**
	*	@brief Return the smaller of two values, or the first value if both are equal.
	*	@param a_val1 is the first value.
	*	@param a_val2 is the second value.
	*	@return const reference to smallest value.
	*/
	template<class T>
	const T& Max(const T& a_val1, const T& a_val2) {
		return !(a_val2 > a_val1) ? a_val1 : a_val2;
	}

	/**
	*	@brief Constrain given value between a range.
	*	@param a_val is the value to constrain.
	*	@param a_upper is the upper range.
	*	@param a_lower is the lower range.
	*	@return const reference to constrained value.
	*/
	template<class T>
	const T& Clamp(const T& a_val, const T& a_upper, const T& a_lower) {
		return Min<T>(Max<T>(a_val, a_lower), a_upper);
	}

}
