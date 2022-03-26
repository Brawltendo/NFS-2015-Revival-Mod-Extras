#pragma once
#include "pch.h"
#include <corecrt_math.h>
#include <xmmintrin.h>
#include "PerformanceModification/PerformanceModification.h"
#include <util/graph.h>
#include "util\utils.h"

enum DriftState
{
	DriftState_Out = 0,
	DriftState_Entering = 1,
	DriftState_In = 2,
	DriftState_Exiting = 3,
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
	// The speed, in MPS, for 
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
	/* mCSYawScaleForZeroSteerTime   */  1.5f,
	/* mYawScaleForZeroSteerTime     */  0.5f,
	/* mCSYawScaleForMaxSteerTime    */  2.5f,
	/* mYawScaleForMaxSteerTime	     */  0.5f,
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
	/* mMaxYawSpeedInDrift	         */  2.f,
	/* mDriftMaintainSpeedAmount     */  9.2f,
	/* mDriftMaintainSpeedScale	     */  1.f,
	/* mLowSpeedForYawAccelScale     */  0.f,
	/* mHighSpeedForYawAccelScale    */  21.4f,
	/* mYawAccelScaleForLowSpeed     */  3.f,
	/* mYawAccelScaleForHighSpeed    */  1.f,
	/* mSteeringThreshold	         */  0.3f,
	/* mBrakeThreshold	             */  0.95f,
	/* mThrottleThreshold	         */  0.95f,
	/* mMinTimeForGasStab	         */  0.1f,
	/* mMaxTimeForGasStab	         */  0.2f,
	/* mOffGasTimeForExitingDrift    */  0.2f,
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
}