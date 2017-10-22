#pragma once

#include "MMath.h"

#define VISCO_INLINE inline

#define VISCO_MAX_CONTACTS 4

#ifdef VISCO_DLL

//DLL
#ifdef VISCOSITY_EXPORTS
#define VISOC_API __declspec(dllexport)
#else

#define VISCO_API __declspec(dllimport)
#endif //Export

#else

//Static Library
#define VISCO_API

#endif //DLL