#include <stdlib.h>
#include "shape.h"

#pragma region Shape_Types

typedef struct plane {
	shape s;
	vec3 normal;
	scalar distance;
} plane;

typedef struct sphere {
	shape s;
	scalar radius;
} sphere;

#pragma endregion Shape_Types

void shapeDestroy(shape *s) {
	free(s);
}

shape* shapeCreatePlane(const vec3 *n, scalar d) {
	plane *ret = (plane*)malloc(sizeof(plane));

	ret->s.type = SHAPE_PLANE;
	ret->s.mass = 0;
	ret->s.restitution = 0.001f;
	ret->s.friction = 0.5f;

	vec3Normalize(&ret->normal, n);

	ret->distance = d;

	return (shape*)ret;
}

static inline void sphereIntertia(sphere *s) {
	scalar inertia = (scalar)(2./5.) * s->s.mass * s->radius * s->radius;
	mat3Diagonal(&s->s.inertiaTensor, inertia);
	mat3Diagonal(&s->s.invInertiaTensor, 1.f / inertia);
}
static inline void sphereMass(sphere *s, scalar density) {
	s->s.mass = (4.f/3.f) * mm_pi * s->radius * s->radius * s->radius * density;
	sphereIntertia(s);
}
shape* shapeCreateSphere(scalar r) {
	sphere *ret = (sphere*)malloc(sizeof(sphere));

	ret->s.type = SHAPE_SPHERE;
	ret->s.restitution = 0.6f;
	ret->s.friction = 0.1f;
	ret->radius = r;
	sphereMass(ret, 1);

	return (shape*)ret;
}

void shapeSetDensity(shape *s, scalar density) {
	switch (s->type) {
	case SHAPE_PLANE:
		return;

	case SHAPE_SPHERE:
		sphereMass((sphere*)s, density);
		return;
	}
}
void shapeRecalcIntertia(shape *s) {
	switch (s->type) {
	case SHAPE_PLANE:
		return;

	case SHAPE_SPHERE:
		sphereIntertia((sphere*)s);
		return;
	}
}

static inline void genSphereAabb(aabb *dest, const sphere *s) {
	*dest = (aabb){
		{-s->radius,-s->radius,-s->radius },
		{ s->radius, s->radius, s->radius }
	};
}

void shapeGenerateAabb(aabb *dest, const shape *s, const quat *rot) {
	switch (s->type) {
	case SHAPE_PLANE:
		*dest = aabbInfinity;
		break;

	case SHAPE_SPHERE:
		genSphereAabb(dest, (sphere*)s);
		break;
	}
}

static inline int collidePlaneSphere(contact *dest, const plane *p, const sphere *b, const vec3 *posb) {
	scalar dist = vec3Dot(posb, &p->normal) - p->distance;

	if (dist > -b->radius && dist < b->radius) {
		dest->normal = p->normal;
		dest->distance = b->radius - dist;
		vec3 normTemp, temp;
		vec3Negate(&normTemp, &p->normal);
		vec3MulScalar(&temp, &normTemp, b->radius);
		vec3Add(&dest->position, &temp, posb);
		return 1;
	} else {
		return 0;
	}
}
static inline int collideSphereSphere(contact *dest, const sphere *a, const vec3 *posa, const sphere *b, const vec3 *posb) {
	vec3 t;
	vec3Sub(&t, posb, posa);

	scalar radius = a->radius + b->radius;
	scalar distance = vec3Length(&t);

	if (distance > radius) {
		return 0;
	} else {
		contact ret;

		if (distance == 0) {
			ret.distance = a->radius * 2;
			ret.normal = vec3YAxis;
			ret.position = *posa;
		} else {
			ret.distance = radius - distance;
			vec3MulScalar(&ret.normal, &t, 1.f / distance);
			vec3 pos;
			vec3MulScalar(&pos, &ret.normal, a->radius);
			vec3Add(&ret.position, posa, &pos);
		}

		*dest = ret;
		return 1;
	}
}
int shapeCollide(contact *dest, size_t maxContacts, const shape *a, const vec3 *posa, const quat *rota,
				 const shape *b, const vec3 *posb, const quat *rotb) {

	switch (a->type) {
	case SHAPE_PLANE:
		switch (b->type) {
		case SHAPE_PLANE:
			return 0;
			break;

		case SHAPE_SPHERE:
			return collidePlaneSphere(dest, (plane*)a, (sphere*)b, posb);
			break;

		default:
			return 0;
			break;
		}
		break;

	case SHAPE_SPHERE:
		switch (b->type) {
		case SHAPE_PLANE:
			return collidePlaneSphere(dest, (plane*)b, (sphere*)a, posa);
			break;

		case SHAPE_SPHERE:
			return collideSphereSphere(dest, (sphere*)a, posa, (sphere*)b, posb);
			break;

		default:
			return 0;
			break;
		}
		break;

	default:
		return 0;
		break;
	}
}