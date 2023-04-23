#pragma once
#include <math/vectormath.h>
#include <vector>
#ifdef _DEBUG
#include "debug/debug.h"
#endif

constexpr auto PI = 3.14159f;
constexpr auto Epsilon = 0.00000011920929f;
constexpr auto Infinity = 3.4028235e38f;
constexpr auto NegInfinity = -3.4028235e38f;

float map(float s, float a1, float a2, float b1, float b2);

// Clamps the input x within the bounds [min, max]
float clamp(float x, float min, float max);

// Clamps the input x within the bounds [0.0f, 1.0f]
float clamp01(float x);

float sign(float in, float scale);
float sign(float in);

float GetSpeedMph(class NFSVehicle* nfsVehicle);
void SetVehicleYaw(class NFSVehicle& nfsVehicle, float originalYaw, float targetYaw, float dT);
void SetVehicleLinearVel(class NFSVehicle& nfsVehicle, Vec4& originalLinVel, Vec4& targetLinVel, float dT);


#pragma region CONVERSIONS

inline
float DegreesToRadians(float degrees)
{
	return degrees * (PI / 180.f);
}

inline
float RadiansToDegrees(float radians)
{
	return radians * (180.f / PI);
}

inline
float DegreesToAngle(const float degrees)
{
	return degrees / 360.f;
}

inline
float RadiansToAngle(const float radians) {
	return radians / (2.f * PI);
}

inline
float MphToMps(const float mph)
{
	return mph * 0.44703001f;
}

inline
float MpsToMph(const float mps)
{
	return mps * 2.2369399f;
}

#pragma endregion CONVERSIONS



#pragma region NFS_NATIVE_FUNCTIONS

typedef void(__fastcall* _AddWorldCOMForceLogged)(class RaceRigidBody* rigidBody, __m128* force, __m128* lfTimeStep);
typedef void(__fastcall* _AddWorldTorqueLogged)(class RaceRigidBody* rigidBody, __m128* torque, __m128* lvfTimeStep);
typedef void(__fastcall* _MaintainDriftSpeed)(class DriftComponent* driftComp, __m128* lvfGasInput, __m128* lvfBrakeInput, __m128* lDirection, class RaceCarPhysicsObject* lpRaceCar, __m128* lvfTimeStep);
typedef void(__fastcall* _UpdateDriftScale)(class DriftComponent* driftComp, 
											__m128* lvfGasInput, 
											__m128* lvfBrakeInput, 
											__m128* lvfSteeringInputIn, class HandbrakeComponent* lpHandbrake, 
											class RaceCarPhysicsObject* lpRaceCar, 
											__m128* lvfTimeStep, 
											__m128* lvfInvTimeStep);
typedef void(__fastcall* _UpdateDriftAngleDegrees)(class DriftComponent* const driftComp);
typedef void(__fastcall* _UpdateSideForce)(class DriftComponent* driftComp, 
											__m128* lvfAbsDriftScale, 
											class RaceCarPhysicsObject* lpRaceCar, 
											__m128* lvfSteeringInput, 
											__m128* lvfAverageSurfaceGripFactor, 
											__m128* lvfTimeStep);
typedef void(__fastcall* _ApplyDamping)(class DriftComponent* driftComp, __m128* lvfAverageSurfaceGripFactor, __m128* lvfDampingScale, __m128* lvfTimeStep);
typedef void(__fastcall* _ApplyDriftForces)(class DriftComponent* const driftComp, 
											__m128* lvfGasInput, 
											__m128* lvfBrakeInput, 
											__m128* lvfSteeringInput, 
											__m128* lvfAbsDriftScale, 
											class HandbrakeComponent* lpHandbrake, 
											class RaceCarPhysicsObject* lpRaceCar, 
											float lfSpeedMPS, 
											__m128* lvfAverageSurfaceGripFactor, 
											__m128* lvfTimeStep, 
											__m128* lvfInvTimeStep);
typedef void(__fastcall* _DampPitchYawRoll)(class RaceRigidBody* chassis, __m128* pitchDampening, __m128* yawDampening, __m128* rollDampening, __m128* lvfTimeStep);
typedef void(__fastcall* _DampLinearVelocityXYZ)(class RaceRigidBody* chassis, __m128* xAxisDampening, __m128* yAxisDampening, __m128* zAxisDampening, __m128* lvfTimeStep);
typedef struct Matrix44* (__fastcall* _GetTransform)(class RaceRigidBody* chassis, struct Matrix44* transformIn);
typedef __m128* (__fastcall* _GetLinearVelocity)(class RaceRigidBody* const chassis, __m128* lvOut);
typedef __m128* (__fastcall* _SetLinearVelocity)(class DynamicPhysicsEntity* const physEnt, const uint16_t* rbIndex, __m128* lv);
typedef __m128* (__fastcall* _GetAngularVelocity)(class RaceRigidBody* const chassis, __m128* avOut);
typedef __m128* (__fastcall* _AddAngularVelocity)(class DynamicPhysicsEntity* const physEnt, const uint16_t* rbIndex, __m128* av);

extern _AddWorldCOMForceLogged AddWorldCOMForceLogged;
extern _AddWorldTorqueLogged AddWorldTorqueLogged;
extern _DampPitchYawRoll DampPitchYawRoll;
extern _DampLinearVelocityXYZ DampLinearVelocityXYZ;
// Returns Frostbite transformation matrix: Side vector = X, up vector = Y, forward vector = Z, location = W
extern _GetTransform RaceRigidBody_GetTransform;
extern _GetLinearVelocity GetLinearVelocity;
// Sets the current entity's linear velocity to the value of lv
extern _SetLinearVelocity SetLinearVelocity;
//typedef void (__fastcall* _SetLinearVelocity)(void* const physEnt, __m128* newLv);
// Sets the linear velocity of this physics entity to the value of newLv
//extern _SetLinearVelocity SetLinearVelocity;
extern _GetAngularVelocity GetAngularVelocity;
// Adds the value of av to the current entity's angular velocity
extern _AddAngularVelocity AddAngularVelocity;
extern _MaintainDriftSpeed MaintainDriftSpeed;
//extern _UpdateDriftScale UpdateDriftScale;
extern _UpdateDriftAngleDegrees UpdateDriftAngleDegrees;
extern _UpdateSideForce UpdateSideForce;
extern _ApplyDriftForces ApplyDriftForces;
extern _ApplyDamping ApplyDamping;

#pragma endregion NFS_NATIVE_FUNCTIONS