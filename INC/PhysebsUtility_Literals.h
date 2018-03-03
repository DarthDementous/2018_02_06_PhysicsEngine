#pragma once

#define B_SHOW_PARTITIONS 1			// Whether or not to show AABBs for each volume in the partition tree
#define B_VOLUME_COLORS 0			// Whether to set object colors to the volume they are contained in

// The limit point before a value will be zeroed
#define EPSILON 0.001f

#define DEFAULT_TIME_STEP 0.01f			// Frame = one hundredth of a second/100fps

#define DEFAULT_MASS 2.f
#define DEFAULT_FRICTION 1.f
#define DEFAULT_RESTITUTION 0.5f
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


