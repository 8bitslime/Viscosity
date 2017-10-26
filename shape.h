#pragma once

#include "aabb.h"
#include "visco_def.h"

typedef enum shapeType {
	SHAPE_PLANE,
	SHAPE_SPHERE,
	SHAPE_CUBE,
	SHAPE_MESH,
	SHAPE_COMPOUND
} shapeType;

typedef struct shape {
	shapeType type;
	scalar mass;
	mat3 inertiaTensor;
	mat3 invInertiaTensor;
	scalar restitution;
	scalar friction;
} shape;

typedef struct contact {
	vec3 position;
	vec3 normal;
	scalar distance;
} contact;

VISCO_API void shapeDestroy(shape* shape);

VISCO_API shape* shapeCreatePlane(const vec3 *normal, scalar distance);
VISCO_API shape* shapeCreateSphere(scalar radius);

VISCO_API void shapeSetDensity(shape *shape, scalar density);
VISCO_API void shapeRecalcIntertia(shape* shape);

VISCO_API void shapeGenerateAabb(aabb *dest, const shape *shape, const quat *rot);

//Tests collision between 2 shapes. returns the amount of contacts.
VISCO_API int shapeCollide(contact *dest, size_t maxContacts, const shape *a, const vec3 *posa, const quat *rota, const shape *b, const vec3 *posb, const quat *rotb);