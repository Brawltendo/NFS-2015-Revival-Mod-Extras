#pragma once
#include "pch.h"
#include <corecrt_math.h>
#include <xmmintrin.h>
#include "PerformanceModification/PerformanceModification.h"
#include "util/utils.h"


namespace fb
{
	// contains an eastl::vector with all spawned/active vehicle jobs
	// that doesn't matter though, so all we want is the first element which happens to be at 0x28
	struct RaceVehicleJobHandler
	{
		// player vehicle is the first element
		static RaceVehicleJobHandler* m_instance;

		void** m_physPreUpdateCallback;
		char pad[0x8];
		uintptr_t start;
		uintptr_t end;
		uint64_t num;
		class NFSVehicle* m_vehicles[64];

	}; static_assert(offsetof(RaceVehicleJobHandler, m_vehicles[0]) == 0x28, "NfsVehicle should be at 0x28");
}

enum DriftState
{
	DriftState_Out = 0,
	DriftState_Entering = 1,
	DriftState_In = 2,
	DriftState_Exiting = 3,
};

enum DriftEntryReason
{
	DriftEntryReason_None,
	DriftEntryReason_SlipAngle,
	DriftEntryReason_Handbraking,
	DriftEntryReason_Braking,
	DriftEntryReason_GasStab
};

struct DriftParameters
{
	// The yaw scale, while countersteering, when the time spent steering in one direction is zero seconds
	float mCSYawScaleForZeroSteerTime;
	// The yaw scale when the time spent steering in one direction is zero seconds
	float mYawScaleForZeroSteerTime;
	// The yaw scale, while countersteering, when the time spent steering in one direction is mMaxSteeringTime
	float mCSYawScaleForMaxSteerTime;
	// The yaw scale when the time spent steering in one direction is mMaxSteeringTime
	float mYawScaleForMaxSteerTime;
	// Scales the yaw acceleration gained from steering
	float mYawAccelScale;
	// Scales how much the car's current slip angle should influence the drift
	float mSlipAngleRatioScale;
	// The maximum amount of time, in seconds, that will be counted when steering in a single direction
	float mMaxSteeringTime;
	// How fast the yaw speed should accelerate on its own relative to the drift scale
	float mYawAutoAccelAmount;
	// Scales countersteer yaw for low slip angles approaching 90 degrees
	float mAutoCSYawScaleForLowSA;
	// Scales countersteer yaw for high slip angles approaching 90 degrees
	float mAutoCSYawScaleForHighSA;
	// Scales mYawAutoAccelAmount when countersteering
	float mCountersteerYawAccelScale;
	// The slip angle needed to fully exit a drift
	float mAngleToExitDrift;
	// The slip angle needed to fully enter a drift
	float mAngleToEnterDrift;
	float mAngleForMidDriftScale;
	// The slip angle to reach in order to use the full drift scale (highest yaw acceleration)
	float mAngleForMaxDriftScale;
	float mSpeedForMidDriftScale;
	// The speed required, in MPS, in order to completely exit a drift
	float mSpeedToExitDrift;
	// The speed, in MPS, at which a controlled drift will not maintain the entry speed
	float mSpeedForZeroSpeedMaintenance;
	// The speed, in MPS, at which a controlled drift will add the maximum amount of speed maintenance
	float mSpeedForFullSpeedMaintenance;
	// The highest angular velocity that the car should reach while drifting
	float mMaxYawSpeedInDrift;
	// The maximum amount of speed to maintain
	float mDriftMaintainSpeedAmount;
	// Scales the amount of speed maintained
	float mDriftMaintainSpeedScale;
	// The speed, in MPS, at which mYawAccelScaleForLowSpeed is used when entering/exiting a drift
	float mLowSpeedForYawAccelScale;
	// The speed, in MPS, at which mYawAccelScaleForHighSpeed is used when entering/exiting a drift
	float mHighSpeedForYawAccelScale;
	// The scale for how fast the yaw speed should accelerate when entering or exiting a drift at low speeds
	float mYawAccelScaleForLowSpeed;
	// The scale for how fast the yaw speed should accelerate when entering or exiting a drift at high speeds
	float mYawAccelScaleForHighSpeed;
	// The minimum steering input required to enter a controlled drift
	float mSteeringThreshold;
	// The minimum brake input required to enter a controlled drift
	float mBrakeThreshold;
	// The minimum throttle input required to enter a controlled drift
	float mThrottleThreshold;
	// The minimum amount of time, in seconds, since the throttle was released and pressed again to enter a controlled drift
	float mMinTimeForGasStab;
	// The maximum amount of time, in seconds, since the throttle was released and pressed again to enter a controlled drift
	float mMaxTimeForGasStab;
	// The amount of time, in seconds, to start exiting a drift after releasing the throttle
	float mOffGasTimeForExitingDrift;
	// Allows the ability to enter a controlled drift when pressing the brakes
	bool mCanEnterDriftWithBrake;
	// Allows the ability to enter a controlled drift after quickly tapping the throttle
	bool mCanEnterDriftWithGasStab;
	// Allows the ability to enter a controlled drift when the car's slip angle is past a certain threshold
	bool mCanEnterDriftWithSlipAngle;
	// Allows the ability to enter a controlled drift when pulling the handbrakes
	bool mCanEnterDriftWithHandbrake;
};

static const DriftParameters s_GlobalDriftParams =
{
	/* mCSYawScaleForZeroSteerTime   */  0.65f,
	/* mYawScaleForZeroSteerTime     */  0.8f,
	/* mCSYawScaleForMaxSteerTime    */  1.f,
	/* mYawScaleForMaxSteerTime	     */  1.f,
	/* mYawAccelScale                */  1.f,
	/* mSlipAngleRatioScale          */  1.f,
	/* mMaxSteeringTime	             */  2.f,
	/* mYawAutoAccelAmount           */  -0.2f,
	/* mAutoCSYawScaleForLowSA       */  0.f,
	/* mAutoCSYawScaleForHighSA      */  0.f,
	/* mCountersteerYawAccelScale    */  1.f,
	/* mAngleToExitDrift	         */  5.f,
	/* mAngleToEnterDrift	         */  13.f,
	/* mAngleForMidDriftScale        */  40.f,
	/* mAngleForMaxDriftScale        */  50.f,
	/* mSpeedForMidDriftScale        */  20.f,
	/* mSpeedToExitDrift	         */  4.f,
	/* mSpeedForZeroSpeedMaintenance */  0.f,
	/* mSpeedForFullSpeedMaintenance */  2.3f,
	/* mMaxYawSpeedInDrift	         */  2.125f,
	/* mDriftMaintainSpeedAmount     */  9.2f,
	/* mDriftMaintainSpeedScale	     */  1.f,
	/* mLowSpeedForYawAccelScale     */  0.f,
	/* mHighSpeedForYawAccelScale    */  21.4f,
	/* mYawAccelScaleForLowSpeed     */  1.65f,
	/* mYawAccelScaleForHighSpeed    */  1.25f,
	/* mSteeringThreshold	         */  0.3f,
	/* mBrakeThreshold	             */  0.95f,
	/* mThrottleThreshold	         */  0.95f,
	/* mMinTimeForGasStab	         */  0.1f,
	/* mMaxTimeForGasStab	         */  0.5f,
	/* mOffGasTimeForExitingDrift    */  0.55f,
	/* mCanEnterDriftWithBrake	     */  false,
	/* mCanEnterDriftWithGasStab     */  false,
	/* mCanEnterDriftWithSlipAngle   */  false,
	/* mCanEnterDriftWithHandbrake   */  true,
};

namespace RevivalDriftComponent
{

	// Checks for whether a controlled drift can be initiated, and sets up any other values that need to be ready before the main drift functions
	void PreUpdate(class NFSVehicle& nfsVehicle, class DriftComponent& driftComp, int& numWheelsOnGround);
	// Helps control the vehicle's yaw via external forces, both in and out of a controlled drift
	void UpdateAutoSteer(class NFSVehicle& nfsVehicle, class DriftComponent& driftComp, int numWheelsOnGround);
	void ResetDrift(DriftComponent& driftComp);
	void Update(class NFSVehicle& nfsVehicle, class DriftComponent& driftComp, int numWheelsOnGround);

	#ifdef _DEBUG
	void PreUpdate_Debug(class NFSVehicle& nfsVehicle);
	#endif

}