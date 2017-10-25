#include <stdlib.h>
#include <stdio.h>
#include "world.h"

typedef enum jointType {
	JOINT_DELETE = 0,
	JOINT_CONTACT
} jointType;
typedef struct joint {
	jointType type;
	bodyID a, b;
} joint;
typedef struct contact_joint {
	joint joint;
	contact contact;
} contact_joint, joint_max;

//World
typedef struct world {

	vec3 gravity;

	
	size_t body_size;
	size_t body_cap;

	size_t* body_empty;
	size_t body_empty_size;

	bodyType *body_type;
	vec3 *body_pos;  //3
	vec3 *body_vel;  //3
	quat *body_rot;  //4
	vec3 *body_avel; //3
	aabb *body_aabb; //6
	shape **body_shape; //array of pointers, shapes are stored separately from worlds

	size_t joint_size;
	size_t joint_cap;

	size_t *joint_empty;
	size_t joint_empty_size;

	joint_max* joints;

} world;

static world* allocateWorld(size_t body_cap, size_t joint_cap) {
	const size_t size = sizeof(world) +						//world
						sizeof(scalar) * 19 * body_cap +	//body data
						(sizeof(shape*) + sizeof(bodyType) + sizeof(size_t)) * body_cap + //body types, shapes, stack
						(sizeof(joint_max) + sizeof(size_t)) * joint_cap; //Joint array and stack
	unsigned char* data = calloc(1, size);

	world* ret = (world*)data;
	ret->gravity.y = -9.8f;
	ret->body_cap  = body_cap;
	ret->joint_cap = joint_cap;

	ret->body_empty  = (size_t*)&data[sizeof(world)];
	ret->body_type   = (bodyType*)&ret->body_empty[body_cap];
	ret->body_pos    = (vec3*)&ret->body_type[body_cap];
	ret->body_vel    = (vec3*)&ret->body_pos[body_cap];
	ret->body_rot    = (quat*)&ret->body_vel[body_cap];
	ret->body_avel   = (vec3*)&ret->body_rot[body_cap];
	ret->body_aabb   = (aabb*)&ret->body_avel[body_cap];
	ret->body_shape  = (shape**)&ret->body_aabb[body_cap];
	ret->joint_empty = (size_t*)&ret->body_shape[body_cap];
	ret->joints      = (joint_max*)&ret->joint_empty[joint_cap];

	return ret;
}
static void copyWorld(world* newWorld, const world* oldWorld) {
	memcpy(newWorld->body_empty, oldWorld->body_empty, oldWorld->body_cap  * sizeof(size_t));
	memcpy(newWorld->body_type,  oldWorld->body_type,  oldWorld->body_cap  * sizeof(bodyType));
	memcpy(newWorld->body_pos,   oldWorld->body_pos,   oldWorld->body_cap  * sizeof(vec3));
	memcpy(newWorld->body_vel,   oldWorld->body_vel,   oldWorld->body_cap  * sizeof(vec3));
	memcpy(newWorld->body_rot,   oldWorld->body_rot,   oldWorld->body_cap  * sizeof(quat));
	memcpy(newWorld->body_avel,  oldWorld->body_avel,  oldWorld->body_cap  * sizeof(vec3));
	memcpy(newWorld->body_aabb,  oldWorld->body_aabb,  oldWorld->body_cap  * sizeof(aabb));
	memcpy(newWorld->body_shape, oldWorld->body_shape, oldWorld->body_cap  * sizeof(shape*));
	memcpy(newWorld->joint_empty,oldWorld->joint_empty,oldWorld->joint_cap * sizeof(size_t));
	memcpy(newWorld->joints,     oldWorld->joints,     oldWorld->joint_cap * sizeof(joint_max));
	newWorld->body_size        = oldWorld->body_size;
	newWorld->body_empty_size  = oldWorld->body_empty_size;
	newWorld->joint_size       = oldWorld->joint_size;
	newWorld->joint_empty_size = oldWorld->joint_empty_size;
}

world* worldCreate(void) {
	world* ret = allocateWorld(4, 4);
	return ret;
}
void worldDestroy(world *w) {
	free(w);
}

//Bodies
bodyID bodyCreate(world** ptr) {
	world *w = *ptr;

	if (w->body_size >= w->body_cap) {
		//Allocate more space
		world* newWorld = allocateWorld(w->body_cap * 2, w->joint_size);
		copyWorld(newWorld, w);
		*ptr = newWorld;
		free(w);
		w = newWorld;
	}

	size_t index;
	if (w->body_empty_size == 0) {
		index = w->body_size;
	} else {
		index = w->body_empty[--w->body_empty_size];
	}

	w->body_type[index]  = BODY_STATIC;
	w->body_pos[index]   = 
	w->body_vel[index]   =
	w->body_avel[index]  = vec3Zero;
	w->body_rot[index]   = quatIndentity;
	w->body_aabb[index]  = (aabb){0};
	w->body_shape[index] = NULL;
	w->body_size++;

	return index;
}
void bodyDestroy(world* w, bodyID b) {
	w->body_type[b] = BODY_DELETE;
	w->body_empty[w->body_empty_size++] = b;
	w->body_size--;
}

void bodySetType(world *w, bodyID b, bodyType t) {
	w->body_type[b] = t;
}
bodyType bodyGetType(world *w, bodyID b) {
	return w->body_type[b];
}

void bodyGetPosition(vec3 *dest, world *w, bodyID b) {
	*dest = w->body_pos[b];
}
void bodySetPosition(world *w, bodyID b, const vec3 *pos) {
	w->body_pos[b] = *pos;
}

void bodyGetTransform(transform *dest, world *w, bodyID b) {
	transform ret = {
		w->body_pos[b],
		vec3Identity,
		w->body_rot[b]
	};
	*dest = ret;
}
void bodyGetMat4(mat4 *dest, world *w, bodyID b) {
	vec3 pos = w->body_pos[b];
	mat4 ret = {
		1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0,
		pos.x, pos.y, pos.z, 1
	};
	mat4 rot;
	quatToMat4(&rot, &w->body_rot[b]);
	mat4Mul(dest, &rot, &ret);
}

void bodySetShape(world *w, bodyID b, shape *s) {
	w->body_shape[b] = s;

	if (s != NULL) {
		aabb newAABB;
		shapeGenerateAabb(&newAABB, w->body_shape[b], &w->body_rot[b]);
		aabbAddVec3(&w->body_aabb[b], &newAABB, &w->body_pos[b]);
	}
}

static inline void applyForce(world *w, bodyID b, const vec3 *pos, const vec3 *force) {
	scalar mass;
	mat3 inertia;

	if (w->body_shape[b] != NULL) {
		mass = (scalar)1.0 / w->body_shape[b]->mass;
		inertia = w->body_shape[b]->interiaTensor;
	} else {
		mass = 1;
		inertia = mat3Identity;
	}

	//Linear velocity
	vec3 linear;
	vec3MulScalar(&linear, force, mass);
	vec3Add(&w->body_vel[b], &w->body_vel[b], &linear);

	//Angular velocity
	vec3 toPos;
	vec3Sub(&toPos, pos, &w->body_pos[b]);
	vec3 cross;
	vec3Cross(&cross, &toPos, force);
	vec3 torque;
	mat3MulVec3(&torque, &w->body_shape[b]->interiaTensor, &cross);
	vec3Add(&w->body_avel[b], &w->body_avel[b], &torque);
}
void bodyApplyForce(world *w, bodyID b, const vec3 *pos, const vec3 *force) {
	applyForce(w, b, pos, force);
}

static inline void velAtPoint(vec3 *dest, world *w, bodyID b, const vec3 *pos) {
	vec3 toPos;
	vec3Sub(&toPos, pos, &w->body_pos[b]);
	vec3 angular;
	vec3Cross(&angular, &w->body_avel[b], &toPos);
	vec3Add(dest, &w->body_vel[b], &angular);
}
void bodyGetVelocityAtPoint(vec3 *dest, world *w, bodyID b, const vec3 *pos) {
	velAtPoint(dest, w, b, pos);
}

//Joints
static inline jointID pushJoint(world **ptr, joint *j) {
	world *w = *ptr;

	if (w->joint_size >= w->joint_cap) {
		//Allocate more space
		world* newWorld = allocateWorld(w->body_cap, w->joint_cap * 2);
		copyWorld(newWorld, w);
		*ptr = newWorld;
		free(w);
		w = newWorld;
	}

	size_t index;
	if (w->joint_empty_size == 0) {
		index = w->joint_size;
	} else {
		index = w->joint_empty[--w->joint_empty_size];
	}

	switch (j->type) {
	case JOINT_CONTACT:
		w->joints[index] = *((contact_joint*)j);
		break;
	}
	w->joint_size++;

	return index;
}
static inline void destroyJoint(world *w, jointID j) {
	w->joints[j].joint.type = JOINT_DELETE;
	w->joint_empty[w->joint_empty_size++] = j;
	w->joint_size--;
}

//Solve joints
static inline void solveContact(world *w, contact_joint *j) {
	bodyID a = j->joint.a;
	bodyID b = j->joint.b;

	shape* sA = w->body_shape[a];
	shape* sB = w->body_shape[b];

	vec3 velA, velB;
	velAtPoint(&velA, w, a, &j->contact.position);
	velAtPoint(&velB, w, b, &j->contact.position);

	vec3 relative;
	vec3Sub(&relative, &velB, &velA);
	scalar contactVel = vec3Dot(&relative, &j->contact.normal);

	if (contactVel > 0) {
		return;
	} else {
		scalar e = (sA->restitution + sB->restitution) * (scalar)0.5 + 1;
		scalar r = e * contactVel * (sA->mass + sB->mass);

		if (w->body_type[a] == BODY_DYNAMIC) {
			vec3 force;
			vec3MulScalar(&force, &j->contact.normal, r);
			applyForce(w, b, &j->contact.position, &force);
		}
		if (w->body_type[b] == BODY_DYNAMIC) {
			vec3 force;
			vec3MulScalar(&force, &j->contact.normal, -r);
			//TODO: add friction
			applyForce(w, b, &j->contact.position, &force);

			//Error handling
			//if (j->contact.distance > 0.0001) {
			//	//TODO: limit amount of error handling
			//	vec3 err;
			//	vec3MulScalar(&err, &j->contact.normal, j->contact.distance);
			//	vec3Add(&w->body_pos[b], &w->body_pos[b], &err);
			//}
		}
	}
}

//Simulation
static inline void integrateVelocity(world *w, scalar dt) {
	vec3 gravDelta;
	vec3MulScalar(&gravDelta, &w->gravity, dt);

	for (size_t i = 0; i < w->body_cap; i++) {
		if (w->body_type[i] > BODY_STATIC) {
			{ //Linear velocity
				vec3 delta;
				vec3MulScalar(&delta, &w->body_vel[i], dt);
				vec3Add(&w->body_pos[i], &w->body_pos[i], &delta);
			}
			{ //Angular velocity
				scalar hdt = dt * (scalar)0.5;
				quat adelta;
				adelta.w = 0;
				adelta.axis = w->body_avel[i];
				quat hw;
				quatMulScalar(&hw, &adelta, hdt);
				quat hwq;
				quatMul(&hwq, &hw, &w->body_rot[i]);
				quat end;
				quatAdd(&end, &w->body_rot[i], &hwq);
				quatNormalize(&w->body_rot[i], &end);
			}

			if (w->body_type[i] == BODY_DYNAMIC) {
				//Gravy
				vec3Add(&w->body_vel[i], &w->body_vel[i], &gravDelta);
			}
		}
	}
}
static inline void recalculateAABB(world *w) {
	for (size_t i = 0; i < w->body_cap; i++) {
		if (w->body_type[i] > BODY_STATIC && w->body_shape[i] != NULL) {
			aabb newAABB;
			shapeGenerateAabb(&newAABB, w->body_shape[i], &w->body_rot[i]);
			aabbAddVec3(&w->body_aabb[i], &newAABB, &w->body_pos[i]);
		}
	}
}
static inline void naive_collision(world **ptr) { // O(n^2) broad phase, hella optimized
	world* w = *ptr;
	for (size_t i = 0; i < (w->body_cap - 1); i++) {
		if (w->body_type[i] != BODY_DELETE && w->body_shape[i]) {

			for (size_t j = i + 1; j < w->body_cap; j++) {
				if (w->body_type[j] != BODY_DELETE && w->body_shape[j]) {

					if (aabbCollideAabb(&w->body_aabb[i], &w->body_aabb[j])) {
						//Possible collision, you could in theory thread this part

						//Collision detection and creating manifold
						contact_joint constraint;
						contact contacts[4];
						int numContacts;

						if (numContacts = shapeCollide(contacts, 4,
							w->body_shape[i], &w->body_pos[i], &w->body_rot[i],
							w->body_shape[j], &w->body_pos[j], &w->body_rot[j])) {

							constraint.joint.type = JOINT_CONTACT;

							//Edge case with plane shape
							if (w->body_shape[j]->type == SHAPE_PLANE) {
								constraint.joint.a = j;
								constraint.joint.b = i;
							} else {
								constraint.joint.a = i;
								constraint.joint.b = j;
							}

							for (int c = 0; c < numContacts; c++) {
								//Collisions detected and contacts generated
								constraint.contact = contacts[i];

								//Add joint for resolution
								pushJoint(ptr, (joint*)&constraint);
								w = *ptr;
							}
						}
					}
				}
			}
		}
	}
}
static inline void solveConstraints(world *w) {
	for (size_t i = 0; i < w->joint_cap; i++) {
		switch (w->joints[i].joint.type) {
		case JOINT_DELETE:
			break;
		case JOINT_CONTACT:
			solveContact(w, (contact_joint*)&w->joints[i]);
			destroyJoint(w, i);
			break;
		}
	}
}

void worldStep(world **w, scalar dt) {
	integrateVelocity(*w, dt);
	recalculateAABB(*w);

	//collision detection
	naive_collision(w);

	//constraints
	solveConstraints(*w);
}