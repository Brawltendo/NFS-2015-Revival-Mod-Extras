#include "pch.h"
#include "DriftComponent.h"
#include "SteeringComponent.h"
#include "HandbrakeComponent.h"
#include "DriftParams.h"
#include "ConfigData.h"
#include "PerformanceModification/PerformanceModification.h"
#include "NFSClasses.h"
#include "util/debug/debug.h"
#include <util/utils.h>


#if !USE_REVIVAL_COMPONENT
namespace fb
{

const Vec4 KF_NO_BRAKE_THRESHOLD(0.25f);
const Vec4 KF_BRAKE_STAB_MAX_TIME(0.6f);
const Vec4 KF_FULL_STEER_LEFT_THRESHOLD(0.9f);
const Vec4 KF_FULL_STEER_RIGHT_THRESHOLD(-0.9f);

//bool DriftComponent::IsDrifting()
//{
//	return mbIsDrifting;
//}

void DriftComponent::EnterDrift(float lfSpeedMPS, Vec4& lvfStartDriftScale)
{
	mbIsDrifting = true;
	mvfDriftScale = lvfStartDriftScale;
	Vec4 vfPropSpeedMaintainAlongZ;
	Vec4 vfPropSpeedMaintainAlongVel;
	vfPropSpeedMaintainAlongZ = mpParams->mOtherParams.m_performanceModificationComponent->GetModifiedValue(ATM_MaintainSpeedInDriftAmount, mpParams->mOtherParams.m_driftConfig->Maintain_entry_speed_amount);
	vfPropSpeedMaintainAlongVel = mpParams->mOtherParams.m_performanceModificationComponent->GetModifiedValue(ATM_ReduceForwardSpeedAmount, mpParams->mOtherParams.m_driftConfig->Reduce_forward_speed_amount);
	mvfMaintainedSpeed = lfSpeedMPS;
	mvfTimeDrifting = 0.f;
	mvfPropSpeedMaintainAlongZ = vfPropSpeedMaintainAlongZ;
	mvfPropSpeedMaintainAlongVel = vfPropSpeedMaintainAlongVel;
	mvfDriftGasLetOffAmount = 0.f;
	mvfCurrentDriftAngle = 0.f;
	mvfSideForceMagnitude = 0.f;
	mvfCounterSteerSideMag = 0.f;
	mvfDriftYawDamping = 0.f;
	mvfPreviousGasInput = 0.f;
}

void DriftComponent::ExitDrift(SteeringComponent& lpSteeringComponent)
{
	mbIsDrifting = false;
	Vec4 vfDriftScaleVsSteeringScale = VecMul(mvfDriftScale, lpSteeringComponent.steeringScale);
	mvfDriftScale = Vec4::kZero;
	mvfLatDriftForceFactor = Vec4::kOne;
	mvfCurrentDriftAngle = Vec4::kZero;
	mvfSideForceMagnitude = Vec4::kZero;
	mvfCounterSteerSideMag = Vec4::kZero;
	mvfTimeSinceExittingDrift = Vec4::kZero;
	mvfTimeWithLowSlipAngle = Vec4::kZero;
	mvbDriftingWithLowSlipAngle = Vec4::kZero;

	Vec4 vfSteerAmountOnExitDrift = mpParams->mExitParams.m_driftConfig->Steering_amount_on_exit_drift;
	Vec4 vfSteerAngleRadians = lpSteeringComponent.wheelSteeringAngleRadians;
	lpSteeringComponent.wheelSteeringAngleRadians = VecSelect(vfSteerAmountOnExitDrift * vfSteerAngleRadians, vfSteerAngleRadians, vfDriftScaleVsSteeringScale < 0.f).x;
	mvfSlipAngleForDriftEntry = mpParams->mDriftTriggerParams.m_performanceModificationComponent->GetModifiedValue(ATM_MinimumAngleForDrift, mpParams->mDriftTriggerParams.m_driftConfig->Slip_angle_to_enter_drift);
}

void DriftComponent::CheckForEnteringDrift(const Vec4& lvfGasInput, const Vec4& lvfBrakeInput, const Vec4& lvfSteeringInput, const Vec4& lvbSteeringUsingWheel, HandbrakeComponent& lpHandbrake, RaceCarPhysicsObject& lpRaceCar, float lfSpeedMPS, const Vec4& lvfTimeStep, const Vec4& lvfInvTimeStep)
{
	Vec4 vbBrakeIsPressed = KF_NO_BRAKE_THRESHOLD < lvfBrakeInput;
	Vec4 vbBrakeIsNotPressed = lvfBrakeInput < 0.1f;
	NFSVehicle& nfsVehicle = **(NFSVehicle**)&lpRaceCar;

	mvfTimePressingBrake += VecSelect(lvfTimeStep, Vec4::kZero, VecAnd(VecNot(vbBrakeIsPressed), VecNot(vbBrakeIsNotPressed)));
	mvfTimePressingBrake += VecSelect(lvfTimeStep, Vec4::kZero, vbBrakeIsPressed);
	mvbPressedFullBrake = VecOr(vbBrakeIsPressed, mvbPressedFullBrake);
	Vec4 vfBrakeStabTime = VecSelect(VecSelect(mvfTimePressingBrake, KF_BRAKE_STAB_MAX_TIME, vbBrakeIsNotPressed), KF_BRAKE_STAB_MAX_TIME, mvbPressedFullBrake);
	mvfTimePressingBrake = VecSelect(Vec4::kZero, mvfTimePressingBrake, vbBrakeIsNotPressed);
	mvbPressedFullBrake = VecSelect(Vec4::kZero, mvbPressedFullBrake, vbBrakeIsNotPressed);

	Vec4 vbIsSteeringLeft = KF_FULL_STEER_LEFT_THRESHOLD < lvfSteeringInput;
	Vec4 vbIsSteeringRight = lvfSteeringInput < KF_FULL_STEER_RIGHT_THRESHOLD;
	// left
	mvfTimeSteerLeft = VecSelect(mvfTimeSteerLeft + lvfTimeStep, Vec4::kZero, vbIsSteeringLeft);
	mvfLastTimeSteerLeft = VecSelect(mvfTimeSteerLeft, mvfLastTimeSteerLeft, vbIsSteeringLeft);
	mvfTimeSinceSteerLeft = VecSelect(Vec4::kZero, mvfTimeSinceSteerLeft + lvfTimeStep, vbIsSteeringLeft);
	// right
	mvfTimeSteerRight = VecSelect(mvfTimeSteerRight + lvfTimeStep, Vec4::kZero, vbIsSteeringRight);
	mvfLastTimeSteerRight = VecSelect(mvfTimeSteerRight, mvfLastTimeSteerRight, vbIsSteeringRight);
	mvfTimeSinceSteerRight = VecSelect(Vec4::kZero, mvfTimeSinceSteerRight + lvfTimeStep, vbIsSteeringRight);

	// already drifting, doesn't need to check for drift entry
	if (mbIsDrifting)
		return;

	int numWheelsOnGround = 0;
	for (int i = 0; i < 4; ++i)
	{
		if (nfsVehicle.wheelLoad(i) >= Epsilon)
			++numWheelsOnGround;
	}

	// original game wants 4 wheels on the ground
	// let's be more forgiving and allow for 2 before locking the player out of a drift
	if (numWheelsOnGround < 2)
		return;

	float speedMph = MpsToMph(nfsVehicle.m_forwardSpeed);
	DriftParams::DriftTriggerParams& triggerParams = mpParams->mDriftTriggerParams;
	if (speedMph <= triggerParams.m_driftConfig->Minimum_speed_to_enter_drift
		|| nfsVehicle.m_wasHandbrakeOnLastUpdate
		|| nfsVehicle.getBurnoutScale() >= 0.1f
		|| mvfTireGrip.x < 0.1f)
		return;

	Vec4 vbIsBrakeStab = vfBrakeStabTime < KF_BRAKE_STAB_MAX_TIME;
	Vec4 vfMinAngleToEnterDrift = triggerParams.m_performanceModificationComponent->GetModifiedValue(ATM_MinimumAngleForDrift, triggerParams.m_driftConfig->Slip_angle_to_enter_drift);
	Vec4 vfAngleToEnterDriftWhenBrakeStab = triggerParams.m_performanceModificationComponent->GetModifiedValue(ATM_SlipAngleToEnterWhenBrakeStab, triggerParams.m_driftConfig->Slip_angle_to_enter_drift_after_brake_stab);
	Vec4 vfAngleToEnterDriftWhenBraking = triggerParams.m_performanceModificationComponent->GetModifiedValue(ATM_SlipAngleToEnterWhenBraking, triggerParams.m_driftConfig->Slip_angle_to_enter_drift_when_braking);
	// select slip angle based on player intent
	mvfSlipAngleForDriftEntry = VecMin(mvfSlipAngleForDriftEntry, VecSelect(vfAngleToEnterDriftWhenBrakeStab, vfMinAngleToEnterDrift, vbIsBrakeStab));
	Vec4 vfAngleToEnterDrift = VecLerp(mvfSlipAngleForDriftEntry, vfAngleToEnterDrift, lvfBrakeInput);
	if (nfsVehicle.m_wasHandbrakeOnLastUpdate)
		vfAngleToEnterDrift = triggerParams.m_performanceModificationComponent->GetModifiedValue(ATM_SlipAngleToEnterWhenHandbraking, triggerParams.m_driftConfig->Slip_angle_to_enter_drift_when_handbraking);
	
	// check for Scandinavian flick
	Vec4 vbHasFlickedRight = VecNot(VecOr(mvfLastTimeSteerLeft < 0.1f, Vec4(0.65f) < mvfLastTimeSteerLeft)); // !(mvfLastTimeSteerLeft < 0.1f || 0.65f < mvfLastTimeSteerLeft)
	vbHasFlickedRight = VecAnd(vbHasFlickedRight, VecAnd(mvfTimeSinceSteerLeft < 0.15f, Vec4::kZero < mvfTimeSteerRight)); // vbHasFlickedRight && (mvfTimeSinceSteerLeft < 0.15f && 0.f < mvfTimeSteerRight)
	Vec4 vbHasFlickedLeft = VecNot(VecOr(Vec4(0.65f) < mvfLastTimeSteerRight, mvfLastTimeSteerRight < 0.1f)); // !(0.65f < mvfLastTimeSteerRight || mvfLastTimeSteerRight < 0.1f)
	vbHasFlickedLeft = VecAnd(vbHasFlickedLeft, VecAnd(mvfTimeSinceSteerRight < 0.15f, Vec4::kZero < mvfTimeSteerLeft)); // vbHasFlickedLeft && (mvfTimeSinceSteerRight < 0.15f && 0.f < mvfTimeSteerLeft)
	Vec4 vbHasFlicked = VecOr(vbHasFlickedRight, vbHasFlickedLeft);
	Vec4 vfAngleToEnterAfterFlick = triggerParams.m_performanceModificationComponent->GetModifiedValue(ATM_SlipAngleToEnterWhenScandinavianFlick, triggerParams.m_driftConfig->Slip_angle_to_enter_drift_after_scandinavian_flick);
	
	// the original game uses the body slip angle instead of the avg rear wheel slip angle
	// it's a really strange change compared to previous games, so here we're gonna revert that
	Vec4 vfAvgRearSlipAngle = RadiansToDegrees((nfsVehicle.m_raceCarOutputState.tireSlipAngle[2] + nfsVehicle.m_raceCarOutputState.tireSlipAngle[3]) * 0.5f);
	if (VecCmpLT(VecSelect(vfAngleToEnterAfterFlick, vfAngleToEnterDrift, vbHasFlicked), VecAbs(vfAvgRearSlipAngle)))
	{
		// entering a drift

		Vec4 vbWasHandbrakeOn = BoolToVecMask(nfsVehicle.m_wasHandbrakeOnLastUpdate);
		Vec4 vfHandbrakeDriftScale;
		lpHandbrake.GetDriftScaleFromHandbrake(vfHandbrakeDriftScale, lpRaceCar);
		Vec4 vfSlipAngleSign = VecSign(vfAvgRearSlipAngle);
		Vec4 vfStartingDriftScale = mpParams->mDriftScaleParams.m_driftConfig->Starting_drift_scale;
		Vec4 vfStartingDriftScaleAfterFlick = mpParams->mDriftScaleParams.m_performanceModificationComponent->GetModifiedValue(ATM_StartingDriftScaleAfterScandinavianFlick, mpParams->mDriftScaleParams.m_driftConfig->Starting_drift_scale_after_scandinavian_flick);
		vfStartingDriftScale = VecSelect(vfStartingDriftScaleAfterFlick, VecSelect(vfHandbrakeDriftScale, vfStartingDriftScale, vbWasHandbrakeOn), vbHasFlicked);

		Vec4 vfDriftScale = VecSelect(vfSlipAngleSign, VecNeg(vfSlipAngleSign), vbHasFlicked) * (vfStartingDriftScale + Vec4::kEpsilon);
		EnterDrift(fmaxf(nfsVehicle.m_forwardSpeed, 0.f), vfDriftScale);
	}
}

void DriftComponent::UpdateDriftState(const Vec4& lvfGasInput, const Vec4& lvfBrakeInput, const Vec4& lvfSteeringInput, const Vec4& lvbSteeringUsingWheel, HandbrakeComponent& lpHandbrake, SteeringComponent& lpSteeringComponent, RaceCarPhysicsObject& lpRaceCar, float lfSpeedMPS, const Vec4& lvfTimeStep, const Vec4& lvfInvTimeStep)
{
	// native functions
	bool(__fastcall* DoAllWheelsHaveTraction)(RaceCarPhysicsObject*) = reinterpret_cast<bool(__fastcall*)(RaceCarPhysicsObject*)>(0x144196040);
	NFSVehicle& nfsVehicle = **(NFSVehicle**)&lpRaceCar;

	CheckForEnteringDrift(lvfGasInput, lvfBrakeInput, lvfSteeringInput, lvbSteeringUsingWheel, lpHandbrake, lpRaceCar, lfSpeedMPS, lvfTimeStep, lvfInvTimeStep);
	if (!mbIsDrifting)
		return;

	// drift should be exited upon meeting any of these conditions
	bool isAirborne = nfsVehicle.m_timeAirborne > 1.f;
	bool doWheelsHaveTraction = DoAllWheelsHaveTraction(&lpRaceCar);
	bool isInReverse = nfsVehicle.m_raceCarOutputState.gear == G_REVERSE;
	// NFS15 doesn't check for collisions here, but it'll fix a couple bugs related to side force and steering
	bool hasCollided = nfsVehicle.m_vehicleState == NFSVehicleState_Collided;
	bool isInNeutral = fabsf(lfSpeedMPS) < 0.5f && nfsVehicle.m_input->m_clutch < 0.5f;
	if (isAirborne
	|| isInReverse
	|| hasCollided
	|| isInNeutral
	|| nfsVehicle.m_wasHandbrakeOnLastUpdate
	|| !doWheelsHaveTraction
	|| mvfTireGrip.x < 0.1f)
	{
		ExitDrift(lpSteeringComponent);
	}
	else if (mbIsDrifting && mvfTimeDrifting.x > 0.5f)
	{
		// at this point, the player is locked in to a drift for 0.5 secs before they can exit or chain another one

		// get avg rear slip angle instead of body slip angle again
		Vec4 vfAvgRearSlipAngle = RadiansToDegrees((nfsVehicle.m_raceCarOutputState.tireSlipAngle[2] + nfsVehicle.m_raceCarOutputState.tireSlipAngle[3]) * 0.5f);
		Vec4 vfAngleToExitDrift = mpParams->mExitParams.m_performanceModificationComponent->GetModifiedValue(ATM_DriftAngleToExitDrift, mpParams->mExitParams.m_driftConfig->Drift_angle_to_exit_drift);
		// drift is still active and not ready to exit, so reset the low slip angle timer
		if (!VecCmpLT(VecAbs(vfAvgRearSlipAngle), vfAngleToExitDrift))
		{
			mvfTimeWithLowSlipAngle = Vec4::kZero;
			return;
		}

		Vec4 vfAbsSteeringInput = VecAbs(lvfSteeringInput);
		Vec4 vfDonutSpeedLimitHighMps = MphToMps(mpParams->mYawTorqueParams.m_driftConfig->Donut_speed_limit_high);
		if (VecCmpEQ(VecAnd(Vec4::kHalf < vfAbsSteeringInput, vfDonutSpeedLimitHighMps < fabsf(lfSpeedMPS)), Vec4::kZero)
		|| !VecCmpLE(Vec4::kHalf, mvfTimeWithLowSlipAngle))
		{
			ExitDrift(lpSteeringComponent);
			return;
		}

		// handle drift chaining

		mvbDriftingWithLowSlipAngle = BoolToVecMask(true);
		Vec4 vfStartingDriftScale = mpParams->mDriftScaleParams.m_driftConfig->Starting_drift_scale;
		Vec4 vfHandbrakeDriftScale;
		lpHandbrake.GetDriftScaleFromHandbrake(vfHandbrakeDriftScale, lpRaceCar);
		Vec4 vfSlipAngleSign = VecSign(vfAvgRearSlipAngle);
		Vec4 vfDriftScale = (VecSelect(vfHandbrakeDriftScale, vfStartingDriftScale, BoolToVecMask(nfsVehicle.m_wasHandbrakeOnLastUpdate)) + Vec4::kEpsilon) * vfSlipAngleSign;
		EnterDrift(fmaxf(nfsVehicle.m_forwardSpeed, 0.f), vfDriftScale);
	}
}

}
#endif // !USE_REVIVAL_COMPONENT
