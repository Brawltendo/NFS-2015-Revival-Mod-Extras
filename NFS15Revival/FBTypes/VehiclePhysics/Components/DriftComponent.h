#pragma once
#include <xmmintrin.h>
#include <math/vectormath.h>


#if !USE_REVIVAL_COMPONENT
// forward declare here to keep out of the fb namespace
class NFSVehicle;
class RaceCarPhysicsObject;

namespace fb
{

class SteeringComponent;
class HandbrakeComponent;

class DriftComponent
{
public:
	static const bool sbDEBUGEnableDrift = true;
	//virtual bool IsDrifting();
	// the game REALLY doesn't like when I try to use these as member functions, so guess we're doing this instead

	void EnterDrift(float lfSpeedMPS, Vec4& lvfStartDriftScale);
	void ExitDrift(SteeringComponent& lpSteeringComponent);
	void CheckForEnteringDrift(
		const Vec4& lvfGasInput,
		const Vec4& lvfBrakeInput,
		const Vec4& lvfSteeringInput,
		const Vec4& lvbSteeringUsingWheel,
		HandbrakeComponent& lpHandbrake,
		RaceCarPhysicsObject& lpRaceCar,
		float lfSpeedMPS,
		const Vec4& lvfTimeStep,
		const Vec4& lvfInvTimeStep);
	void UpdateDriftState(
		const Vec4& lvfGasInput,
		const Vec4& lvfBrakeInput,
		const Vec4& lvfSteeringInput,
		const Vec4& lvbSteeringUsingWheel,
		HandbrakeComponent& lpHandbrake,
		SteeringComponent& lpSteeringComponent,
		RaceCarPhysicsObject& lpRaceCar,
		float lfSpeedMPS,
		const Vec4& lvfTimeStep,
		const Vec4& lvfInvTimeStep);

//private:

	char pad_0000[16];
	enum EExitDriftReason_DEBUG
	{
		eDriftingExitReasonNone_DEBUG,
		eDriftingExitReasonWorldCollision_DEBUG,
		eDriftingExitReasonRaceCarCollision_DEBUG,
		eDriftingExitReasonTrafficCollision_DEBUG,
		eDriftingExitReasonTooSlow_DEBUG,
		eDriftingExitReasonInAir_DEBUG,
		eDriftingExitReasonStraightenedUp_DEBUG,
		eDriftingExitReasonInNeutral_DEBUG,
		eDriftingExitReasonInReverse_DEBUG,
		eDriftingExitReasonHandbrake_DEBUG,
		eDriftingExitReasonDriftScaleZero_DEBUG,
		eDriftingExitReasonCrashed_DEBUG,
		eDriftingExitReasonCount
	} meDriftExitReason_DEBUG; // this is unused in NFS15

	bool mbIsDrifting;
	bool mbIsCounterSteeringInDrift;
	class RaceRigidBody* mpChassisRigidBody;
	class DriftParams* mpParams;
	Vec4 mv_DriftYawTorque_NaturalYawTorque_Spare_Spare;
	Vec4 mvfDriftGasLetOffAmount;
	Vec4 mvfTimeSinceExittingDrift;
	Vec4 mvfMaintainedSpeed;
	Vec4 mvfDriftYawDamping;
	Vec4 mvfDriftScale;
	Vec4 mvfTimeDrifting;
	Vec4 mvfSideForceMagnitude;
	Vec4 mvfPropSpeedMaintainAlongZ;
	Vec4 mvfPropSpeedMaintainAlongVel;
	Vec4 mvfLatDriftForceFactor;
	Vec4 mvfCurrentDriftAngle;
	Vec4 mvfTimeInDriftWithStaticFriction;
	Vec4 mvfCounterSteerSideMag;
	Vec4 mvfRearSlipAngle;
	Vec4 mv_YawDamping_Spare_Spare_Spare;
	Vec4 mv_MaintainSpeedForce_SideForce_Spare_Spare;
	Vec4 mvfPreviousGasInput;
	Vec4 mvfMaxSlipAngle;
	Vec4 mvfSlipAngleForDriftEntry;
	Vec4 mvfTimePressingBrake;
	Vec4 mvbPressedFullBrake;

	// NFS15 additions

	Vec4 mvfTimeWithLowSlipAngle;
	Vec4 mvbDriftingWithLowSlipAngle;
	Vec4 mvfTimeSteerLeft;
	Vec4 mvfLastTimeSteerLeft;
	Vec4 mvfTimeSinceSteerLeft;
	Vec4 mvfTimeSteerRight;
	Vec4 mvfLastTimeSteerRight;
	Vec4 mvfTimeSinceSteerRight;
	Vec4 mvfTireGrip;

	class PerformanceModificationComponent* m_performanceModificationComponent;
}; //static_assert(offsetof(DriftComponent, mbIsDrifting) == 0x14, "Wrong offset");

}
#else
class DriftComponent
{
	// any comments that aren't offsets tell what a field is being used for in the custom implementation

public:
	char pad_0000[20]; //0x0000
	bool isDrifting; //0x0014
	bool counterSteeringInDrift; //0x0015

	// 0x0016
	bool isExitingDrift;

	// 0x0017
	// current drift state
	char pad_0017;

	class RaceRigidBody* mpChassisRigidBody; //0x0018
	class DriftParams* mpParams; //0x0020
	char pad_0028[8]; //0x0028

	// 0x0030
	// current yaw speed (local Y angular vel)
	float currentYawTorque;

	// 0x0034
	// drift entry reason
	int someEnum;
	char pad_0038[24]; //0x0038

	// 0x50
	// index 0: time since exiting drift
	// index 1: drift exit timer
	__m128 mvfTimeSinceExittingDrift;

	// 0x0060
	// index 0: the drift direction
	// index 1: drift entry yaw accel scale
	__m128 mvfMaintainedSpeed;

	// 0x0070
	// index 0: yaw control
	__m128 mvfDriftYawDamping;
	// 0x0080
	__m128 mvfDriftScale;
	__m128 mvfTimeDrifting; //0x0090

	// 0x00A0
	// force from last update
	__m128 mvfSideForceMagnitude;
	__m128 mvfPropSpeedMaintainAlongZ; //0x00B0
	__m128 mvfPropSpeedMaintainAlongVel; //0x00C0
	__m128 mvfLatDriftForceFactor; //0x00D0

	// 0x00E0
	// index 0: current angle
	// index 1: max angle reached
	__m128 mvfCurrentDriftAngle;

	__m128 mvfTimeInDriftWithStaticFriction; //0x00F0
	__m128 mvfCounterSteerSideMag; //0x0100
	__m128 mvfRearSlipAngle; //0x0110
	__m128 mv_YawDamping_Spare_Spare_Spare; //0x0120
	__m128 mv_MaintainSpeedForce_SideForce_Spare_Spare; //0x0130

	// 0x0140
	// index 0: time since throttle was released
	__m128 mvfPreviousGasInput;

	__m128 mvfMaxSlipAngle; //0x0150
	__m128 mvfSlipAngleForDriftEntry; //0x0160
	__m128 mvfTimePressingBrake; //0x0170
	__m128 mvbPressedFullBrake; //0x0180
	__m128 mvfTimeWithLowSlipAngle; //0x0190
	__m128 mvbDriftingWithLowSlipAngle; //0x01A0

	// 0x01B0
	// index 0: time steering in a single direction
	// index 1: current steering value during drift
	__m128 mvfTimeSteerLeft;

	__m128 mvfLastTimeSteerLeft; //0x01C0
	__m128 mvfTimeSinceSteerLeft; //0x01D0
	__m128 mvfTimeSteerRight; //0x01E0
	__m128 mvfLastTimeSteerRight; //0x01F0
	__m128 mvfTimeSinceSteerRight; //0x0200
	__m128 mvfTireGrip; //0x0210
	class PerformanceModificationComponent* m_performanceModificationComponent; //0x0220
};
#endif // !USE_REVIVAL_COMPONENT