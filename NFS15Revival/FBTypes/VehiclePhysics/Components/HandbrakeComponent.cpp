#include "HandbrakeComponent.h"
#include "ConfigData.h"
#include "PerformanceModification/PerformanceModification.h"
#include "NFSClasses.h"
#include <util/utils.h>


const Vec4 KVF_HANDBRAKE_SCALE_LOW_SPEED(40.f);
const Vec4 KVF_HANDBRAKE_SCALE_HIGH_SPEED(95.f);
Vec4& fb::HandbrakeComponent::GetDriftScaleFromHandbrake(Vec4& lvfHandbrakeScaleOut, const RaceCarPhysicsObject& lpRaceCar)
{
	NFSVehicle& nfsVehicle = **(NFSVehicle**)&lpRaceCar;

	Vec4 vfHandbrakeDriftScaleAtLowSpeed = perfModComponent->GetModifiedValue(ATM_DriftScaleFromHandbrakeAtLowSpeed, driftConfig->DriftScaleFromHandbrakeAtLowSpeed);
	Vec4 vfHandbrakeDriftScaleAtHighSpeed = perfModComponent->GetModifiedValue(ATM_DriftScaleFromHandbrakeAtHighSpeed, driftConfig->DriftScaleFromHandbrakeAtHighSpeed);
	Vec4 vfRoadSpeedMph = MpsToMph(nfsVehicle.m_forwardSpeed);
	Vec4 vfSpeedRatio = VecRamp(vfRoadSpeedMph, KVF_HANDBRAKE_SCALE_LOW_SPEED, KVF_HANDBRAKE_SCALE_HIGH_SPEED);
	lvfHandbrakeScaleOut = VecLerp(vfHandbrakeDriftScaleAtLowSpeed, vfHandbrakeDriftScaleAtHighSpeed, vfSpeedRatio);
	return lvfHandbrakeScaleOut;
}
