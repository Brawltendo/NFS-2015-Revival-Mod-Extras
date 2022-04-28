#pragma once
#include <math/vectormath.h>
#include <vector>
#include "NFSClasses.h"
#ifdef _DEBUG
#include "debug/debug.h"
#endif

constexpr auto PI = 3.14159f;
constexpr auto Epsilon = 0.00000011920929;

float map(float s, float a1, float a2, float b1, float b2);
// Clamps the input x within the bounds [min, max]
float clamp(float x, float min, float max);
// Clamps the input x within the bounds [0.0f, 1.0f]
float clamp01(float x);
PointGraph8 initPointGraph8(__m128 curveData[10]);
float EvaluatePointGraph8(PointGraph8* pgIn, float xVal);
float sign(float in, float scale);
float sign(float in);
float GetSpeedMph(class NFSVehicle* nfsVehicle);

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


// anonymous namespace so the compiler stops complaining and these can actually be used in other places without having to redefine them
namespace
{
	#pragma region NFS_NATIVE_FUNCTIONS

	typedef PointGraph8* (__fastcall* _initPointGraph8FromCurveData)(PointGraph8* pointGraphIn, __m128 (*curveData)[10]);
	typedef float(__fastcall* _PointGraph8__Evaluate)(int pgCount, float(*pgInX)[8], float(*pgInY)[8], float xVal);
	typedef void(__fastcall* _AddWorldCOMForceLogged)(RaceRigidBody* rigidBody, __m128* force, __m128* lfTimeStep);
	typedef void(__fastcall* _AddWorldTorqueLogged)(RaceRigidBody* rigidBody, __m128* torque, __m128* lvfTimeStep);
	typedef void(__fastcall* _MaintainDriftSpeed)(DriftComponent* driftComp, __m128* lvfGasInput, __m128* lvfBrakeInput, __m128* lDirection, class RaceCarPhysicsObject* lpRaceCar, __m128* lvfTimeStep);
	typedef void(__fastcall* _UpdateDriftScale)(DriftComponent* driftComp, __m128* lvfGasInput, __m128* lvfBrakeInput, __m128* lvfSteeringInputIn, HandbrakeComponent* lpHandbrake, class RaceCarPhysicsObject* lpRaceCar, __m128* lvfTimeStep, __m128* lvfInvTimeStep);
	typedef void(__fastcall* _UpdateDriftAngleDegrees)(DriftComponent* const driftComp);
	typedef void(__fastcall* _UpdateSideForce)(DriftComponent* driftComp, __m128* lvfAbsDriftScale, class RaceCarPhysicsObject* lpRaceCar, __m128* lvfSteeringInput, __m128* lvfAverageSurfaceGripFactor, __m128* lvfTimeStep);
	typedef void(__fastcall* _ApplyDamping)(DriftComponent* driftComp, __m128* lvfAverageSurfaceGripFactor, __m128* lvfDampingScale, __m128* lvfTimeStep);
	typedef void(__fastcall* _ApplyDriftForces)(DriftComponent* const driftComp, __m128* lvfGasInput, __m128* lvfBrakeInput, __m128* lvfSteeringInput, __m128* lvfAbsDriftScale, HandbrakeComponent* lpHandbrake, class RaceCarPhysicsObject* lpRaceCar, float lfSpeedMPS, __m128* lvfAverageSurfaceGripFactor, __m128* lvfTimeStep, __m128* lvfInvTimeStep);

	_initPointGraph8FromCurveData initPointGraph8FromCurveData = (_initPointGraph8FromCurveData)0x1441B7BF0;
	_PointGraph8__Evaluate PointGraph8__Evaluate = (_PointGraph8__Evaluate)0x143F6A8E0;
	_AddWorldCOMForceLogged AddWorldCOMForceLogged = (_AddWorldCOMForceLogged)0x144170EE0;
	_AddWorldTorqueLogged AddWorldTorqueLogged = (_AddWorldTorqueLogged)0x144170F50;

	typedef void(__fastcall* _DampPitchYawRoll)(RaceRigidBody* chassis, __m128* pitchDampening, __m128* yawDampening, __m128* rollDampening, __m128* lvfTimeStep);
	_DampPitchYawRoll DampPitchYawRoll = (_DampPitchYawRoll)0x1441712D0;

	typedef void(__fastcall* _DampLinearVelocityXYZ)(RaceRigidBody* chassis, __m128* xAxisDampening, __m128* yAxisDampening, __m128* zAxisDampening, __m128* lvfTimeStep);
	_DampLinearVelocityXYZ DampLinearVelocityXYZ = (_DampLinearVelocityXYZ)0x144170FF0;

	typedef Matrix44* (__fastcall* _GetTransform)(RaceRigidBody* chassis, Matrix44* transformIn);
	// Returns Frostbite transformation matrix: Side vector = X, up vector = Y, forward vector = Z, location = W
	_GetTransform RaceRigidBody_GetTransform = (_GetTransform)0x1441718C0;

	typedef __m128* (__fastcall* _GetLinearVelocity)(RaceRigidBody* const chassis, __m128* lvOut);
	_GetLinearVelocity GetLinearVelocity = (_GetLinearVelocity)0x144171830;

	typedef __m128* (__fastcall* _SetLinearVelocity)(class DynamicPhysicsEntity* const physEnt, const uint16_t* rbIndex, __m128* lv);
	// Sets the current entity's linear velocity to the value of lv
	_SetLinearVelocity SetLinearVelocity = (_SetLinearVelocity)0x143ADAE80;

	//typedef void (__fastcall* _SetLinearVelocity)(void* const physEnt, __m128* newLv);
	// Sets the linear velocity of this physics entity to the value of newLv
	//_SetLinearVelocity SetLinearVelocity = (_SetLinearVelocity)0x143AD4FE0;

	typedef __m128* (__fastcall* _GetAngularVelocity)(RaceRigidBody* const chassis, __m128* avOut);
	_GetAngularVelocity GetAngularVelocity = (_GetAngularVelocity)0x1441717E0;

	typedef __m128* (__fastcall* _AddAngularVelocity)(class DynamicPhysicsEntity* const physEnt, const uint16_t* rbIndex, __m128* av);
	// Adds the value of av to the current entity's angular velocity
	_AddAngularVelocity AddAngularVelocity = (_AddAngularVelocity)0x143ADA400;

	_MaintainDriftSpeed MaintainDriftSpeed = (_MaintainDriftSpeed)0x1441960E0;
	//_UpdateDriftScale UpdateDriftScale = (_UpdateDriftScale)0x144197840;
	_UpdateDriftAngleDegrees UpdateDriftAngleDegrees = (_UpdateDriftAngleDegrees)0x144197690;
	_UpdateSideForce UpdateSideForce = (_UpdateSideForce)0x144197FC0;
	_ApplyDriftForces ApplyDriftForces = (_ApplyDriftForces)0x144193890;
	_ApplyDamping ApplyDamping = (_ApplyDamping)0x1441935E0;

	#pragma endregion NFS_NATIVE_FUNCTIONS

	inline void SetVehicleYaw(NFSVehicle& nfsVehicle, float originalYaw, float targetYaw, float dT)
	{	
		// DampPitchYawRoll has been patched to add angular velocity instead of multiplying the original by the damping values
		// since I have no idea where the native function to set angular velocity is, let's just do it this way now that this function is only called here
		// this way it'll just add the difference between the current and target angular vel, giving the same result anyway
		vec4 newYaw(targetYaw - originalYaw);
		vec4 timestep(dT);
		__m128* zero = (__m128*)&vec4::s_Zero.simdValue;
		DampPitchYawRoll(nfsVehicle.m_rigidBodyInterface, zero, &newYaw.simdValue, zero, &timestep.simdValue);
	}

	inline void SetVehicleLinearVel(NFSVehicle& nfsVehicle, vec4& originalLinVel, vec4& targetLinVel, float dT)
	{
		vec4 newLv(targetLinVel - originalLinVel);
		vec4 timestep(dT);
		vec4 x = newLv.x;
		vec4 y = newLv.y;
		vec4 z = newLv.z;
		DampLinearVelocityXYZ(nfsVehicle.m_rigidBodyInterface, &x.simdValue, &y.simdValue, &z.simdValue, &timestep.simdValue);
	}

}