#include "pch.h"
#include "utils.h"
#include "FBTypes\NFSClasses.h"


#pragma region NFS_NATIVE_FUNCTIONS

_AddWorldCOMForceLogged AddWorldCOMForceLogged = reinterpret_cast<_AddWorldCOMForceLogged>(0x144170EE0);
_AddWorldTorqueLogged AddWorldTorqueLogged = reinterpret_cast<_AddWorldTorqueLogged>(0x144170F50);
_DampPitchYawRoll DampPitchYawRoll = reinterpret_cast<_DampPitchYawRoll>(0x1441712D0);
_DampLinearVelocityXYZ DampLinearVelocityXYZ = reinterpret_cast<_DampLinearVelocityXYZ>(0x144170FF0);
// Returns Frostbite transformation matrix: Side vector = X, up vector = Y, forward vector = Z, location = W
_GetTransform RaceRigidBody_GetTransform = reinterpret_cast<_GetTransform>(0x1441718C0);
_GetLinearVelocity GetLinearVelocity = reinterpret_cast<_GetLinearVelocity>(0x144171830);
// Sets the current entity's linear velocity to the value of lv
_SetLinearVelocity SetLinearVelocity = reinterpret_cast<_SetLinearVelocity>(0x143ADAE80);
//typedef void (__fastcall* _SetLinearVelocity)(void* const physEnt, __m128* newLv);
// Sets the linear velocity of this physics entity to the value of newLv
//_SetLinearVelocity SetLinearVelocity = reinterpret_cast<_SetLinearVelocity>(0x143AD4FE0);
_GetAngularVelocity GetAngularVelocity = reinterpret_cast<_GetAngularVelocity>(0x1441717E0);
// Adds the value of av to the current entity's angular velocity
_AddAngularVelocity AddAngularVelocity = reinterpret_cast<_AddAngularVelocity>(0x143ADA400);
_MaintainDriftSpeed MaintainDriftSpeed = reinterpret_cast<_MaintainDriftSpeed>(0x1441960E0);
//_UpdateDriftScale UpdateDriftScale = reinterpret_cast<_UpdateDriftScale>(0x144197840);
_UpdateDriftAngleDegrees UpdateDriftAngleDegrees = reinterpret_cast<_UpdateDriftAngleDegrees>(0x144197690);
_UpdateSideForce UpdateSideForce = reinterpret_cast<_UpdateSideForce>(0x144197FC0);
_ApplyDriftForces ApplyDriftForces = reinterpret_cast<_ApplyDriftForces>(0x144193890);
_ApplyDamping ApplyDamping = reinterpret_cast<_ApplyDamping>(0x1441935E0);

#pragma endregion NFS_NATIVE_FUNCTIONS


float map(float in, float inMin, float inMax, float outMin, float outMax)
{
	return outMin + (in - inMin) * (outMax - outMin) / (inMax - inMin);
}

float clamp(float x, float min, float max)
{
	return fminf(fmaxf(x, min), max);
}

float clamp01(float x)
{
	return fminf(fmaxf(x, 0.f), 1.f);
}

float sign(float in, float scale)
{
	if (in > 0)
		return 1 * fabsf(scale);
	if (in < 0)
		return -1 * fabsf(scale);
	else
		return 0;
}

float sign(float in)
{
	if (in > 0.f)
		return 1.f;
	if (in < 0.f)
		return -1.f;
	else
		return 0.f;
}

float GetSpeedMph(NFSVehicle* nfsVehicle)
{
	return nfsVehicle->m_forwardSpeed * 2.2369399f;
}

void SetVehicleYaw(NFSVehicle& nfsVehicle, float originalYaw, float targetYaw, float dT)
{
	// DampPitchYawRoll has been patched to add angular velocity instead of multiplying the original by the damping values
	// since I have no idea where the native function to set angular velocity is, let's just do it this way now that this function is only called here
	// this way it'll just add the difference between the current and target angular vel, giving the same result anyway
	vec4 newYaw(targetYaw - originalYaw);
	vec4 timestep(dT);
	__m128* zero = (__m128*) & vec4::s_Zero.simdValue;
	DampPitchYawRoll(nfsVehicle.m_rigidBodyInterface, zero, &newYaw.simdValue, zero, &timestep.simdValue);
}

void SetVehicleLinearVel(NFSVehicle& nfsVehicle, vec4& originalLinVel, vec4& targetLinVel, float dT)
{
	vec4 newLv(targetLinVel - originalLinVel);
	vec4 timestep(dT);
	vec4 x = newLv.x;
	vec4 y = newLv.y;
	vec4 z = newLv.z;
	DampLinearVelocityXYZ(nfsVehicle.m_rigidBodyInterface, &x.simdValue, &y.simdValue, &z.simdValue, &timestep.simdValue);
}
