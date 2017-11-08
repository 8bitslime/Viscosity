#pragma once

#include "visco_def.h"
#include "shape.h"

typedef struct world world;
typedef size_t bodyID;
typedef size_t jointID;

typedef enum bodyType {
	BODY_DELETE = 0,
	BODY_STATIC,
	BODY_DYNAMIC,
	BODY_KINEMATIC
} bodyType;

VISCO_API world* worldCreate(void);
VISCO_API void   worldDestroy(world *world);

VISCO_API void worldStep(world **world, scalar delta);

VISCO_API bodyID bodyCreate(world **world);
VISCO_API void   bodyDestroy(world *world, bodyID body);

VISCO_API void     bodySetType(world *world, bodyID body, bodyType type);
VISCO_API bodyType bodyGetType(world *world, bodyID body);

VISCO_API void bodyGetPosition(vec3 *dest, world *world, bodyID body);
VISCO_API void bodySetPosition(world *world, bodyID body, const vec3 *position);

VISCO_API void bodyGetOrientation(quat *dest, world *world, bodyID body);
VISCO_API void bodySetOrientation(world *world, bodyID body, const quat *rot);

VISCO_API void bodyGetTransform(transform *dest, world *world, bodyID body);
VISCO_API void bodyGetMat4(mat4 *dest, world *world, bodyID body);

VISCO_API void bodySetShape(world *world, bodyID body, shape *shape);

VISCO_API void bodyApplyForce(world *world, bodyID body, const vec3 *pos, const vec3 *force);
VISCO_API void bodyApplyForceAtCenter(world *world, bodyID body, const vec3 *force);
VISCO_API void bodyApplyTorque(world *world, bodyID body, const vec3 *torque);

VISCO_API void bodyGetVelocityAtPoint(vec3 *dest, world *world, bodyID body, const vec3 *pos);

//VISCO_API void jointDestroy(world *world, jointID joint);