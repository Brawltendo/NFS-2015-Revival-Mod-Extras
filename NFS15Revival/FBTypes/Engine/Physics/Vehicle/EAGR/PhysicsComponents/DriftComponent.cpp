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

const Vec4 KVF_NO_BRAKE_THRESHOLD(0.25f);
const Vec4 KVF_BRAKE_STAB_MAX_TIME(0.6f);
const Vec4 KVF_FULL_STEER_LEFT_THRESHOLD(0.9f);
const Vec4 KVF_FULL_STEER_RIGHT_THRESHOLD(-0.9f);
// The min amount of time to steer in the initial flick direction
const Vec4 KVF_FLICK_MIN_TIME_STEERING(0.1f);
// The max amount of time to steer in the initial flick direction
const Vec4 KVF_FLICK_MAX_TIME_STEERING(0.65f);
// How much time is allowed to pass from the moment the first flick input is released
// i.e.: the amount of time since the player has steered left before they start steering right
const Vec4 KVF_FLICK_MAX_TIME_SINCE_LAST_STEER(0.15f);
const Vec4 KVF_STARTING_SCALE_FROM_DRIFT_CHAINING(0.1f);

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
	mvbDriftingWithLowSlipAngle = BoolToVecMask(false);

	Vec4 vfSteerAmountOnExitDrift = mpParams->mExitParams.m_driftConfig->Steering_amount_on_exit_drift;
	Vec4 vfSteerAngleRadians = lpSteeringComponent.wheelSteeringAngleRadians;
	lpSteeringComponent.wheelSteeringAngleRadians = VecSelect(vfSteerAmountOnExitDrift * vfSteerAngleRadians, vfSteerAngleRadians, vfDriftScaleVsSteeringScale < 0.f).x;
	mvfSlipAngleForDriftEntry = mpParams->mDriftTriggerParams.m_performanceModificationComponent->GetModifiedValue(ATM_MinimumAngleForDrift, mpParams->mDriftTriggerParams.m_driftConfig->Slip_angle_to_enter_drift);
}

void DriftComponent::CheckForEnteringDrift(const Vec4& lvfGasInput, const Vec4& lvfBrakeInput, const Vec4& lvfSteeringInput, const Vec4& lvbSteeringUsingWheel, HandbrakeComponent& lpHandbrake, RaceCarPhysicsObject& lpRaceCar, float lfSpeedMPS, const Vec4& lvfTimeStep, const Vec4& lvfInvTimeStep)
{
	Vec4 vbBrakeIsPressed = KVF_NO_BRAKE_THRESHOLD < lvfBrakeInput;
	Vec4 vbBrakeIsNotPressed = lvfBrakeInput < 0.1f;
	NFSVehicle& nfsVehicle = **(NFSVehicle**)&lpRaceCar;

	mvfTimePressingBrake += VecSelect(lvfTimeStep, Vec4::kZero, VecAnd(VecNot(vbBrakeIsPressed), VecNot(vbBrakeIsNotPressed)));
	mvfTimePressingBrake += VecSelect(lvfTimeStep, Vec4::kZero, vbBrakeIsPressed);
	mvbPressedFullBrake = VecOr(vbBrakeIsPressed, mvbPressedFullBrake);
	Vec4 vfBrakeStabTime = VecSelect(VecSelect(mvfTimePressingBrake, KVF_BRAKE_STAB_MAX_TIME, vbBrakeIsNotPressed), KVF_BRAKE_STAB_MAX_TIME, mvbPressedFullBrake);
	mvfTimePressingBrake = VecSelect(Vec4::kZero, mvfTimePressingBrake, vbBrakeIsNotPressed);
	mvbPressedFullBrake = VecSelect(Vec4::kZero, mvbPressedFullBrake, vbBrakeIsNotPressed);

	Vec4 vbIsSteeringLeft = KVF_FULL_STEER_LEFT_THRESHOLD < lvfSteeringInput;
	Vec4 vbIsSteeringRight = lvfSteeringInput < KVF_FULL_STEER_RIGHT_THRESHOLD;
	// left
	mvfTimeSteerLeft = VecSelect(mvfTimeSteerLeft + lvfTimeStep, Vec4::kZero, vbIsSteeringLeft);
	mvfLastTimeSteerLeft = VecSelect(mvfTimeSteerLeft, mvfLastTimeSteerLeft, vbIsSteeringLeft);
	mvfTimeSinceSteerLeft = VecSelect(Vec4::kZero, mvfTimeSinceSteerLeft + lvfTimeStep, vbIsSteeringLeft);
	// right
	mvfTimeSteerRight = VecSelect(mvfTimeSteerRight + lvfTimeStep, Vec4::kZero, vbIsSteeringRight);
	mvfLastTimeSteerRight = VecSelect(mvfTimeSteerRight, mvfLastTimeSteerRight, vbIsSteeringRight);
	mvfTimeSinceSteerRight = VecSelect(Vec4::kZero, mvfTimeSinceSteerRight + lvfTimeStep, vbIsSteeringRight);

	int numWheelsOnGround = 0;
	for (int i = 0; i < 4; ++i)
	{
		if (nfsVehicle.wheelLoad(i) >= Epsilon)
			++numWheelsOnGround;
	}

	DriftParams::DriftTriggerParams& triggerParams = mpParams->mDriftTriggerParams;
	float speedMph = MpsToMph(nfsVehicle.m_forwardSpeed);
	// original game wants 4 wheels on the ground
	// let's be more forgiving and allow for 2 before locking the player out of a drift
	bool isAirborne = numWheelsOnGround < 2;
	// NFS15 doesn't check for collisions here, but it'll fix a couple bugs related to side force and steering
	bool hasCollided = nfsVehicle.m_vehicleState == NFSVehicleState_Collided;
	if (speedMph <= triggerParams.m_driftConfig->Minimum_speed_to_enter_drift
	|| mbIsDrifting
	|| nfsVehicle.m_wasHandbrakeOnLastUpdate
	|| isAirborne
	|| hasCollided
	|| nfsVehicle.getBurnoutScale() >= 0.1f
	|| mvfTireGrip.x < 0.1f)
	{
		return;
	}

	Vec4 vbIsBrakeStab = vfBrakeStabTime < KVF_BRAKE_STAB_MAX_TIME;
	Vec4 vfMinAngleToEnterDrift = triggerParams.m_performanceModificationComponent->GetModifiedValue(ATM_MinimumAngleForDrift, triggerParams.m_driftConfig->Slip_angle_to_enter_drift);
	Vec4 vfAngleToEnterDriftWhenBrakeStab = triggerParams.m_performanceModificationComponent->GetModifiedValue(ATM_SlipAngleToEnterWhenBrakeStab, triggerParams.m_driftConfig->Slip_angle_to_enter_drift_after_brake_stab);
	Vec4 vfAngleToEnterDriftWhenBraking = triggerParams.m_performanceModificationComponent->GetModifiedValue(ATM_SlipAngleToEnterWhenBraking, triggerParams.m_driftConfig->Slip_angle_to_enter_drift_when_braking);
	// select slip angle based on player intent
	mvfSlipAngleForDriftEntry = VecMin(mvfSlipAngleForDriftEntry, VecSelect(vfAngleToEnterDriftWhenBrakeStab, vfMinAngleToEnterDrift, vbIsBrakeStab));
	Vec4 vfAngleToEnterDrift = VecLerp(mvfSlipAngleForDriftEntry, vfAngleToEnterDrift, lvfBrakeInput);
	if (nfsVehicle.m_wasHandbrakeOnLastUpdate)
		vfAngleToEnterDrift = triggerParams.m_performanceModificationComponent->GetModifiedValue(ATM_SlipAngleToEnterWhenHandbraking, triggerParams.m_driftConfig->Slip_angle_to_enter_drift_when_handbraking);
	
	// check for Scandinavian flick
	Vec4 vbWantsToFlickRight = VecNot(VecOr(mvfLastTimeSteerLeft < KVF_FLICK_MIN_TIME_STEERING, mvfLastTimeSteerLeft > KVF_FLICK_MAX_TIME_STEERING));
	vbWantsToFlickRight = VecAnd(vbWantsToFlickRight, VecAnd(mvfTimeSinceSteerLeft < KVF_FLICK_MAX_TIME_SINCE_LAST_STEER, mvfTimeSteerRight > Vec4::kZero));
	Vec4 vbWantsToFlickLeft = VecNot(VecOr(mvfLastTimeSteerRight < KVF_FLICK_MIN_TIME_STEERING, mvfLastTimeSteerRight > KVF_FLICK_MAX_TIME_STEERING));
	vbWantsToFlickLeft = VecAnd(vbWantsToFlickLeft, VecAnd(mvfTimeSinceSteerRight < KVF_FLICK_MAX_TIME_SINCE_LAST_STEER, mvfTimeSteerLeft > Vec4::kZero));
	Vec4 vbIsFlicking = VecOr(vbWantsToFlickRight, vbWantsToFlickLeft);
	Vec4 vfAngleToEnterAfterFlick = triggerParams.m_performanceModificationComponent->GetModifiedValue(ATM_SlipAngleToEnterWhenScandinavianFlick, triggerParams.m_driftConfig->Slip_angle_to_enter_drift_after_scandinavian_flick);
	
	// the original game uses the body slip angle instead of the avg rear wheel slip angle
	// it's a really strange change compared to previous games, so here we're gonna revert that
	Vec4 vfAvgRearSlipAngle = RadiansToDegrees((nfsVehicle.m_raceCarOutputState.tireSlipAngle[2] + nfsVehicle.m_raceCarOutputState.tireSlipAngle[3]) * 0.5f);
	//if (VecCmpLT(VecSelect(vfAngleToEnterAfterFlick, vfAngleToEnterDrift, vbIsFlicking), VecAbs(vfAvgRearSlipAngle)))
	if (VecCmpLT(vfAngleToEnterDrift, VecAbs(vfAvgRearSlipAngle)))
	{
		// entering a drift

		Vec4 vbWasHandbrakeOn = BoolToVecMask(nfsVehicle.m_wasHandbrakeOnLastUpdate);
		Vec4 vfHandbrakeDriftScale = lpHandbrake.GetDriftScaleFromHandbrake(lpRaceCar);
		Vec4 vfSlipAngleSign = VecSign(vfAvgRearSlipAngle);
		Vec4 vfStartingDriftScale = mpParams->mDriftScaleParams.m_driftConfig->Starting_drift_scale;
		Vec4 vfStartingDriftScaleAfterFlick = mpParams->mDriftScaleParams.m_performanceModificationComponent->GetModifiedValue(ATM_StartingDriftScaleAfterScandinavianFlick, mpParams->mDriftScaleParams.m_driftConfig->Starting_drift_scale_after_scandinavian_flick);
		//vfStartingDriftScale = VecSelect(vfStartingDriftScaleAfterFlick, VecSelect(vfHandbrakeDriftScale, vfStartingDriftScale, vbWasHandbrakeOn), vbIsFlicking);
		vfStartingDriftScale = VecSelect(vfHandbrakeDriftScale, vfStartingDriftScale, vbWasHandbrakeOn);

		//Vec4 vfDriftScale = VecSelect(vfSlipAngleSign, VecNeg(vfSlipAngleSign), vbIsFlicking) * (vfStartingDriftScale + Vec4::kEpsilon);
		Vec4 vfDriftScale = VecNeg(vfSlipAngleSign) * (vfStartingDriftScale + Vec4::kEpsilon);
		EnterDrift(fmaxf(nfsVehicle.m_forwardSpeed, 0.f), vfDriftScale);
	}

	mvfSlipAngleForDriftEntry += VecMul(lvfTimeStep, triggerParams.m_driftConfig->Brake_stab_return_speed);
	mvfSlipAngleForDriftEntry = VecMin(mvfSlipAngleForDriftEntry, vfMinAngleToEnterDrift);
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
		// check drift scale to fix bug where the car will get stuck in a loop entering/exiting a low scale drift every 0.5 secs
		if (VecCmpLT(VecAbs(vfAvgRearSlipAngle), vfAngleToExitDrift) || VecCmpLE(VecAbs(mvfDriftScale), Vec4::kEpsilon))
		{
			// exiting or chaining drift

			//ExitDrift(lpSteeringComponent);

			Vec4 vfAbsSteeringInput = VecAbs(lvfSteeringInput);
			Vec4 vfDonutSpeedLimitHighMps = MphToMps(mpParams->mYawTorqueParams.m_driftConfig->Donut_speed_limit_high);
			if (!(VecCmpGT(vfAbsSteeringInput, Vec4::kHalf) && VecCmpGT(fabsf(lfSpeedMPS), vfDonutSpeedLimitHighMps))
			|| VecCmpGE(mvfTimeWithLowSlipAngle, Vec4::kHalf))
			{
				ExitDrift(lpSteeringComponent);
			}
			else
			{
				// added in NFS15 to assist chaining drifts, expanded upon in following games
				// in here, it honestly causes more trouble than it's worth
				// it often causes cars to lock up when transitioning from a drift into another drift
				mvbDriftingWithLowSlipAngle = BoolToVecMask(true);
				Vec4 vfStartingDriftScale = mpParams->mDriftScaleParams.m_driftConfig->Starting_drift_scale;
				// the handbrake drift scale will never be used here because they already exit the drift and return above if the handbrake is engaged
				// I have no idea why they put that here when all that was needed was the starting drift scale
				//Vec4 vfHandbrakeDriftScale = lpHandbrake.GetDriftScaleFromHandbrake(lpRaceCar);
				//Vec4 vfSlipAngleSign = VecSign(vfAvgRearSlipAngle);
				//Vec4 vfDriftScale = (VecSelect(vfHandbrakeDriftScale, vfStartingDriftScale, BoolToVecMask(nfsVehicle.m_wasHandbrakeOnLastUpdate)) + Vec4::kEpsilon) * vfSlipAngleSign;

				LinearTransform trans = mpChassisRigidBody->GetTransform();
				Vec4 vfAngVel = mpChassisRigidBody->GetAngularVelocity();
				Vec4 vfDirection = VecSign(VecDot3(vfAngVel, SimdToVec4(trans.up)));
				Vec4 vfDriftScale = (vfStartingDriftScale + Vec4::kEpsilon) * vfDirection;
				EnterDrift(fmaxf(nfsVehicle.m_forwardSpeed, 0.f), vfDriftScale);
			}
		}
		else
		{
			mvfTimeWithLowSlipAngle = Vec4::kZero;
		}
	}
}

void DriftComponent::ApplyDamping(const Vec4& lvfAverageSurfaceGripFactor, const Vec4& lvfDampingScale, const Vec4& lvfTimeStep)
{
	typedef void(__fastcall* FuncSig)(DriftComponent*, const Vec4&, const Vec4&, const Vec4&);
	FuncSig native = reinterpret_cast<FuncSig>(0x1441935E0);
	native(this, lvfAverageSurfaceGripFactor, lvfDampingScale, lvfTimeStep);
}

void DriftComponent::ApplyDriftForces(const Vec4& lvfGasInput, const Vec4& lvfBrakeInput, const Vec4& lvfSteeringInput, const Vec4& lvfAbsDriftScale, HandbrakeComponent& lpHandbrake, RaceCarPhysicsObject& lpRaceCar, float lfSpeedMPS, const Vec4& lvfAverageSurfaceGripFactor, const Vec4& lvfTimeStep, const Vec4& lvfInvTimeStep)
{
	typedef void(__fastcall* FuncSig)(DriftComponent*, const Vec4&, const Vec4&, const Vec4&, const Vec4&, HandbrakeComponent&, RaceCarPhysicsObject&, float, const Vec4&, const Vec4&, const Vec4&);
	FuncSig native = reinterpret_cast<FuncSig>(0x144193890);
	native(this, lvfGasInput, lvfBrakeInput, lvfSteeringInput, lvfAbsDriftScale, lpHandbrake, lpRaceCar, lfSpeedMPS, lvfAverageSurfaceGripFactor, lvfTimeStep, lvfInvTimeStep);
}

void DriftComponent::UpdateDrift(const Vec4& lvfGasInput, const Vec4& lvfBrakeInput, const Vec4& lvfSteeringInput, const Vec4& lvbSteeringUsingWheel, HandbrakeComponent& lpHandbrake, RaceCarPhysicsObject& lpRaceCar, SteeringComponent& lpSteeringComponent, SteeringParams& lpSteeringParams, const Vec4& lvfAverageSurfaceGripFactor, const Vec4& lvfTimeStep, const Vec4& lvfInvTimeStep)
{
	NFSVehicle& nfsVehicle = **(NFSVehicle**)&lpRaceCar;
	// none of Criterion/Ghost's additions outside of the main vehicle sim are substepped
	// let's see what happens if this is substepped
	Vec4 vfSubstepDt = lvfTimeStep.x / nfsVehicle.m_raceCarInputState.numberOfSubSteps;
	Vec4 vfInvSubstepDt = Vec4::kOne / vfSubstepDt;
	// this is what MW12 uses as the steering input into this function
	Vec4 vfSteering = clamp(nfsVehicle.m_input->m_yaw / (RadiansToDegrees(nfsVehicle.m_raceCar->mSteeringResultState.steeringRangeLeft) / nfsVehicle.m_raceCar->mVehicleTuning.maxSteeringAngle), -1.f, 1.f);

	for (int step = 0; step < nfsVehicle.m_raceCarInputState.numberOfSubSteps; ++step)
	{
		mv_DriftYawTorque_NaturalYawTorque_Spare_Spare = Vec4::kZero;
		mv_MaintainSpeedForce_SideForce_Spare_Spare = Vec4::kZero;
		mbIsCounterSteeringInDrift = false;
		mv_YawDamping_Spare_Spare_Spare = Vec4::kZero;
		Vec4 vfAbsDriftScale = VecAbs(mvfDriftScale);
		Vec4 vfLinearVel = mpChassisRigidBody->GetLinearVelocity();
		Vec4 vfFwdSpeed = VecLength3(vfLinearVel);
		// get avg rear slip angle instead of body slip angle again
		mvfRearSlipAngle = RadiansToDegrees((nfsVehicle.m_raceCarOutputState.tireSlipAngle[2] + nfsVehicle.m_raceCarOutputState.tireSlipAngle[3]) * 0.5f);
		mvfMaxSlipAngle = VecMax(mvfMaxSlipAngle, VecAbs(mvfRearSlipAngle));
		
		if (sbDEBUGEnableDrift)
			UpdateDriftState(lvfGasInput, lvfBrakeInput, vfSteering, lvbSteeringUsingWheel, lpHandbrake, lpSteeringComponent, lpRaceCar, vfFwdSpeed.x, vfSubstepDt, vfInvSubstepDt);

		// update drift damping
		Vec4 vfTimeToBlendDamping = Vec4::kOne - (mvfTimeSinceExittingDrift / VecMax(mpParams->mExitParams.m_driftConfig->Time_to_blend_damping, Vec4::kEpsilon));
		Vec4 vfAngleForFullDampingFade = mpParams->mOtherParams.m_performanceModificationComponent->GetModifiedValue(ATM_SlipAngleForFullDampingFade, mpParams->mOtherParams.m_driftConfig->Slip_angle_for_full_damping_fade);
		Vec4 vfFadeRatioVsThrottle = VecClamp(VecAbs(mvfRearSlipAngle) / vfAngleForFullDampingFade, Vec4::kZero, Vec4::kOne) * lvfGasInput;
		// in NFS15, they introduced a "damping fade" (which, based on how they tuned the cars, goes unused in every game)
		// basically it's supposed to "fade" out the amount of damping as the rear slip approaches Slip_angle_for_full_damping_fade
		Vec4 vfDampingScale = VecClamp(vfTimeToBlendDamping - VecSelect(vfFadeRatioVsThrottle, Vec4::kZero, vfAngleForFullDampingFade > Vec4::kZero), Vec4::kZero, Vec4::kOne);
		if (vfDampingScale.x > 0.f)
			ApplyDamping(lvfAverageSurfaceGripFactor, vfDampingScale, vfSubstepDt);

		if (mbIsDrifting)
		{
			mvfTimeDrifting += vfSubstepDt;
			if (VecCmpEQ(mvbDriftingWithLowSlipAngle, BoolToVecMask(true)))
				mvfTimeWithLowSlipAngle += vfSubstepDt;

			int numWheelsOnGround = 0;
			for (int i = 0; i < 4; ++i)
			{
				if (nfsVehicle.wheelLoad(i) >= Epsilon)
					++numWheelsOnGround;
			}

			// NFS15 wants all 4 wheels to be grounded to apply drift forces, later games only require 2 wheels on the ground
			// causes a lot less pain when curbs and sharp elevation changes are at play
			if (numWheelsOnGround >= 2)
			{
				ApplyDriftForces(lvfGasInput, lvfBrakeInput, vfSteering, vfAbsDriftScale, lpHandbrake, lpRaceCar, vfFwdSpeed.x, lvfAverageSurfaceGripFactor, vfSubstepDt, vfInvSubstepDt);
				mvfTimeSinceExittingDrift = Vec4::kZero;
			}
		}
		else
		{
			mvfTimeSinceExittingDrift += vfSubstepDt;
			mvfMaxSlipAngle = Vec4::kZero;
		}
		mvfPreviousGasInput = lvfGasInput;
	}
}

}
#endif // !USE_REVIVAL_COMPONENT
