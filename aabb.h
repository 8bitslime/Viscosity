#pragma once

#include "visco_def.h"

typedef struct aabb {
	vec3 min, max;
} aabb;

static const aabb aabbInfinity = {
	{-INFINITY, -INFINITY, -INFINITY},
	{ INFINITY,  INFINITY,  INFINITY}
};

VISCO_INLINE void aabbAddVec3(aabb *dest, const aabb *a, const vec3 *b) {
	vec3Add(&dest->min, &a->min, b);
	vec3Add(&dest->max, &a->max, b);
}
VISCO_INLINE void aabbSubVec3(aabb *dest, const aabb *a, const vec3 *b) {
	vec3Sub(&dest->min, &a->min, b);
	vec3Sub(&dest->max, &a->max, b);
}
VISCO_INLINE void aabbAdd(aabb *dest, const aabb *a, const aabb *b) {
	*dest = (aabb){
		{ mm_min(a->min.x, b->min.x), mm_min(a->min.y, b->min.y), mm_min(a->min.z, b->min.z) },
		{ mm_max(a->max.x, b->max.x), mm_max(a->max.y, b->max.y), mm_max(a->max.z, b->max.z) }
	};
}

VISCO_INLINE void aabbCenter(vec3 *dest, const aabb *a) {
	vec3 add;
	vec3Add(&add, &a->max, &a->min);
	vec3MulScalar(dest, &add, 0.5f);
}

VISCO_INLINE int aabbCollidePoint(const aabb *a, const vec3 *b) {
	return (b->x >= a->min.x && b->x <= a->max.x) &&
		   (b->y >= a->min.y && b->y <= a->max.y) &&
		   (b->z >= a->min.z && b->z <= a->max.z);
}
VISCO_INLINE int aabbCollideAabb(const aabb *a, const aabb *b) {
	return (a->min.x <= b->max.x && a->max.x >= b->min.x) &&
		   (a->min.y <= b->max.y && a->max.y >= b->min.y) &&
		   (a->min.z <= b->max.z && a->max.z >= b->min.z);
}