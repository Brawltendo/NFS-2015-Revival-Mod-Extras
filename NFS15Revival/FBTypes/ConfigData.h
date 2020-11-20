#pragma once
#include <xmmintrin.h>
#include "NFSEnums.h"

using namespace std;

class GearData
{
public:
	float GearRatio; //0x0000
	float GearUpRPM; //0x0004
	float GearDownRPM; //0x0008
	float GearUpSpeed; //0x000C
	float GearEfficiency; //0x0010
}; //Size: 0x0014

class RaceVehicleABSConfigData
{
public:
	char pad_0000[16]; //0x0000
	char* Name; //0x0010
	float ABSSlipRatioSystemTargetSR; //0x0018
	float ABSTorqueRatioLowerBound; //0x001C
	__m128* ABSTorqueRatioVsSpeed; //0x0020
	float ABSTorqueRatioUpperBound; //0x0028
	bool ABSSlipRatioSystemEnable; //0x002C
}; //Size: 0x002D

class RaceVehicleAerodynamicsConfigData
{
public:
	char pad_0000[16]; //0x0000
	char* Name; //0x0010
	__m128* AeroCG; //0x0018
	__m128* AeroCoefficient; //0x0020
	float AeroCoefficient90DegLoss; //0x0028
	float AeroCoefficientMaxGroundEffect; //0x002C
	float AeroCoefficientMaxLift; //0x0030
	float AeroCoefficientOffThrottle; //0x0034
	float DragCoefficient; //0x0038
	float NitrousDragCoefficient; //0x003C
	float TopSpeed; //0x0040
	float NitrousTopSpeed; //0x0044
	float GroundEffectHeightRatio; //0x0048
	float LiftHeightRatio; //0x004C
	float FlyingCarAeroCoef_BigAir; //0x0050
	float FlyingCarAeroCoef_SmallAir; //0x0054
	float Downforce; //0x0058
	float DownforceOffset; //0x005C
	float DownforceOffsetUnderBraking; //0x0060
	float DownforceOffsetInDrift; //0x0064
}; //Size: 0x0068

class LongGripCurve
{
public:
	float SpinRatioForPeakGrip; //0x0000
	float PeakGrip; //0x0004
	float SpinRatioForFinalGrip; //0x0008
	float FinalGrip; //0x000C
}; //Size: 0x0010

class LateralGripCurve
{
public:
	float SlipForPeakGrip; //0x0000
	float InitialGripRate; //0x0004
	float SlipAngleForPeakGrip; //0x0008
	float PeakGrip; //0x000C
}; //Size: 0x0010

class SlipAngleInfluenceOnLongGrips
{
public:
	float MinimumSlipAngle; //0x0000
	float LargeSlipAngle; //0x0004
	float MaximumLateralInfluence; //0x0008
	float LateralInfluencePower; //0x000C
}; //Size: 0x0010

class SpinRatioInfluenceOnLateralGrips
{
public:
	float MinimumSpinRatio; //0x0000
	float LargeSpinRatio; //0x0004
	float CopLargeSpinRatio; //0x0008
	float MaximumLongInfluence; //0x000C
	float LongInfluencePower; //0x0010
}; //Size: 0x0014

class RaceVehicleAnalyticalTireConfigData
{
public:
	char pad_0000[16]; //0x0000
	char* Name; //0x0010
	float TorqueRatioToSlipRatioConstant; //0x0018
	float GripFactor; //0x001C
	float SlidingGripFactor; //0x0020
	float WheelLoadTireResponse; //0x0024
	float LongitudinalBias; //0x0028
	float AvailableGrip; //0x002C
	LongGripCurve LongGripforZeroSlipAngle; //0x0030
	LongGripCurve LongGripforLargeSlipAngles; //0x0040
	LongGripCurve BrakingLongGripforZeroSlipAngle; //0x0050
	LongGripCurve BrakingLongGripforLargeSlipAngles; //0x0060
	LateralGripCurve LateralGripforZeroSpinRatio; //0x0070
	LateralGripCurve LateralGripforLargeSpinRatio; //0x0080
	LateralGripCurve BrakingLateralGripforLargeSpinRatio; //0x0090
	SlipAngleInfluenceOnLongGrips TractionSlipAngleInfluenceOnLongGrips; //0x00A0
	SlipAngleInfluenceOnLongGrips BrakingSlipAngleInfluenceOnLongGrips; //0x00B0
	SpinRatioInfluenceOnLateralGrips TractionSpinRatioInfluenceOnLateralGrips; //0x00C0
	SpinRatioInfluenceOnLateralGrips BrakingSpinRatioInfluenceOnLateralGrips; //0x00D4
	float CamberAngleForMaximumGrip; //0x00E8
	float CamberAngleForMinimumGrip; //0x00EC
	float CamberMinimumGrip; //0x00F0
	float LateralRelaxationLength; //0x00F4
	float MaximumLateralDeflection; //0x00F8
	float LateralFrictionScale; //0x00FC
	float LongitudinalFrictionScale; //0x0100
}; //Size: 0x0104

class RaceVehicleBrakesConfigData
{
public:
	char pad_0000[16]; //0x0000
	char* Name; //0x0010
	__m128* FrontBrakeTorqueVsSpeed; //0x0018
	__m128* RearBrakeTorqueVsSpeed; //0x0020
	__m128* HandbrakeTorqueVsSpeed; //0x0028
	float FrontBrakeOffset; //0x0030
	float RearBrakeOffset; //0x0034
	float TimeForFullBrakeStrength; //0x0038
	float MinHandbrakeTime; //0x003C
	float FrontBrakeScalarWhenSteering; //0x0040
	float RearBrakeScalarWhenSteering; //0x0044
	float BrakeScalarDeadZone; //0x0048
	float BackwardsExtraBrakeStrength; //0x004C
}; //Size: 0x0050

class RaceVehicleChassisConfigData
{
public:
	char pad_0000[16]; //0x0000
	char* Name; //0x0010
	float CollisionInvMassScale; //0x0018
	char pad_001C[4]; //0x001C
	__m128 InertiaBoxCollision; //0x0020
	__m128 InertiaBoxTumbling; //0x0030
	__m128 InertiaBoxVehiclePhysics; //0x0040
	__m128 TensorScale; //0x0050
	__m128 TumbleLocalCOG; //0x0060
	float FrontAxle; //0x0070
	char pad_0074[4]; //0x0074
	__m128* FrontWeightBias; //0x0078
	float HealthScalar; //0x0080
	float Mass; //0x0084
	__m128* RollCenter; //0x0088
	float TrackWidthFront; //0x0090
	float TrackWidthRear; //0x0094
	float WheelBase; //0x0098
	float GravityScale; //0x009C
	float VehicleVsVehicleFriction; //0x00A0
	float VehicleVsVehicleBounceScalar; //0x00A4
	float SideSwipeSpeedMatchingScalar; //0x00A8
	float PitManeuverVictimLateralGripScalar; //0x00AC
}; //Size: 0x00B0

class VehicleInputData
{
public:
	float ThrottleDeadzone; //0x0000
	float BrakeDeadzone; //0x0004
	float YawDeadzone; //0x0008
	float PitchDeadzone; //0x000C
	float RollDeadzone; //0x0010
	float ThrottleInertiaOutDuration; //0x0014
	float ThrottleInertiaInDuration; //0x0018
	float ThrottleInertiaMinRatio; //0x001C
	float BrakeInertiaOutDuration; //0x0020
	float BrakeInertiaInDuration; //0x0024
	float BrakeInertiaMinRatio; //0x0028
	float YawInertiaOutDuration; //0x002C
	float YawInertiaInDuration; //0x0030
	float YawInertiaMinRatio; //0x0034
	float PitchInertiaOutDuration; //0x0038
	float PitchInertiaInDuration; //0x003C
	float PitchInertiaMinRatio; //0x0040
	float RollInertiaOutDuration; //0x0044
	float RollInertiaInDuration; //0x0048
	float RollInertiaMinRatio; //0x004C
}; //Size: 0x0050

class RaceVehicleConfigData
{
public:
	char pad_0000[16]; //0x0000
	char* Name; //0x0010
	VehicleMode VehicleModeAtReset; //0x0018
	float VehicleModeChangeEnteringTime; //0x001C
	float VehicleModeChangeStartingTime; //0x0020
	float VehicleScoringTopSpeedMPH; //0x0024
	float VehicleModeChangeStoppingTime; //0x0028
	float VehicleModeChangeLeavingTime; //0x002C
	class RaceVehiclePerformanceModifierData* PerformanceModifierData[1]; //0x0030
	class PerformanceUpgradeContainer* Item1Upgrades[1]; //0x0038
	class PerformanceUpgradeContainer* Item2Upgrades[1]; //0x0040
	class PerformanceUpgradeContainer* Item3Upgrades[1]; //0x0048
	class PerformanceUpgradeContainer* Item4Upgrades[1]; //0x0050
	class PerformanceUpgradeContainer* Item5Upgrades[1]; //0x0058
	class RaceVehicleTuningAssets* TuningAssets; //0x0060
	class RaceVehicleForcedInductionUpgradesData* ForcedInductionUpgrades; //0x0068
	class RaceVehiclePerformanceModifiersData* PerformanceModifiers; //0x0070
	VehicleInputData Input; //0x0078
	class RaceVehicleSteeringConfigData* Steering; //0x00C8
	class RaceVehicleSteeringWheelConfigData* SteeringWheel; //0x00D0
	class RaceVehicleEngineConfigData* Engine; //0x00D8
	class RaceVehicleForcedInductionConfigData* ForcedInduction; //0x00E0
	class RaceVehicleTransmissionConfigData* Transmission; //0x00E8
	class RaceVehicleAerodynamicsConfigData* Aerodynamics; //0x00F0
	class RaceVehicleChassisConfigData* Chassis; //0x00F8
	class RaceVehicleSuspensionConfigData* Suspension; //0x0100
	class RaceVehicleBrakesConfigData* Brakes; //0x0108
	class RaceVehicleDriftConfigData* Drift; //0x0110
	class RaceVehicleABSConfigData* ABS; //0x0118
	class RaceVehicleESCConfigData* ESC; //0x0120
	class RaceVehicleTCSConfigData* TCS; //0x0128
	class RaceVehicleTireConfigData* Tire; //0x0130
	class RaceVehicleTumbleConfigData* Tumble; //0x0138
	class RaceVehicleXCarConfigData* XCar; //0x0140
	float StaticCollisionBreakCollisionMod; //0x0148
	float StaticCollisionBreakVelocityMod; //0x014C
}; //Size: 0x0150

class RaceVehicleDriftConfigData
{
public:
	char pad_0000[16]; //0x0000
	char* Name; //0x0010
	float Minimum_speed_to_enter_drift; //0x0018
	float Slip_angle_to_enter_drift; //0x001C
	float Slip_angle_to_enter_drift_when_handbraking; //0x0020
	float Slip_angle_to_enter_drift_when_braking; //0x0024
	float Slip_angle_to_enter_drift_after_brake_stab; //0x0028
	float Slip_angle_to_enter_drift_after_scandinavian_flick; //0x002C
	float Brake_stab_return_speed; //0x0030
	float Drift_angle_to_exit_drift; //0x0034
	float Steering_amount_on_exit_drift; //0x0038
	float Slip_angle_to_start_blending_down_drift_scale; //0x003C
	float Slip_angle_for_zero_drift_scale; //0x0040
	float Time_to_blend_damping; //0x0044
	float Starting_drift_scale; //0x0048
	float Starting_drift_scale_after_scandinavian_flick; //0x004C
	float Drift_scale_from_handbrake; //0x0050
	float Drift_scale_decay; //0x0054
	float Drift_scale_from_steering; //0x0058
	float Drift_scale_from_braking; //0x005C
	float Drift_scale_from_counter_steering; //0x0060
	float Drift_scale_from_gas_let_off; //0x0064
	float Drift_scale_from_gas_stab; //0x0068
	float Drift_angle_for_full_gas_influence; //0x006C
	float DriftScaleFromHandbrakeAtLowSpeed; //0x0070
	float DriftScaleFromHandbrakeAtHighSpeed; //0x0074
	float Initial_yaw_torque; //0x0078
	float Minimum_yaw_torque; //0x007C
	float Mid_yaw_torque; //0x0080
	float Maximum_yaw_torque; //0x0084
	float Gas_let_off_yaw_torque; //0x0088
	float Donut_yaw_torque; //0x008C
	float Donut_speed_limit_low; //0x0090
	float Donut_speed_limit_high; //0x0094
	float Default_steering_remapping; //0x0098
	float Counter_steering_remapping; //0x009C
	float Side_force_magnitude; //0x00A0
	float Drift_scale_for_maximum_side_force; //0x00A4
	float Speed_for_no_sideforce; //0x00A8
	float Speed_for_maximum_side_force; //0x00AC
	float Minimum_drift_angle_for_side_force; //0x00B0
	float Drift_angle_for_side_force; //0x00B4
	float Drift_angle_for_decay; //0x00B8
	float Decay_rate; //0x00BC
	float Drift_sideways_damping; //0x00C0
	float Min_decay_side_force_magnitude; //0x00C4
	float Side_force_multiplier; //0x00C8
	float Drift_angular_damping; //0x00CC
	float Maintain_entry_speed_amount; //0x00D0
	float Max_speed_difference_for_maintain_speed; //0x00D4
	float Reduce_forward_speed_amount; //0x00D8
	float Max_countersteering; //0x00DC
	float Default_steering; //0x00E0
	float Max_countersteering_at_low_slip_angle; //0x00E4
	float Max_countersteering_at_high_slip_angle; //0x00E8
	float Low_slip_angle_for_counter_steering; //0x00EC
	float High_slip_angle_for_counter_steering; //0x00F0
	float Slip_angle_for_deep_drift; //0x00F4
	float Max_counter_steering_power; //0x00F8
	float Slip_angle_for_full_damping_fade; //0x00FC
	float ExtraFishTailTorque; //0x0100
	char pad_0104[4]; //0x0104
	__m128 (*YawDampeningAtAngle)[10]; //0x0108
	__m128 (*YawDampeningAtSpeed)[10]; //0x0110
	float TimeForMaxDonutGripEffect; //0x0118
	float DonutLongitudinalGripMin; //0x011C
	float DonutLongitudinalGripMax; //0x0120
	float DonutLateralGripMin; //0x0124
	float DonutLateralGripMax; //0x0128
}; //Size: 0x012C

class RaceVehicleEngineConfigData
{
public:
	char pad_0000[16]; //0x0000
	char* Name; //0x0010
	float FlyWheelMass; //0x0018
	float Idle; //0x001C
	float EngineAntibogMult; //0x0020
	float EngineAntibogRampTime; //0x0024
	__m128* EngineBrakingVsGear; //0x0028
	__m128* EngineFrictionTorque; //0x0030
	float EngineResistance; //0x0038
	float MaxRpm; //0x003C
	float RedLine; //0x0040
	char pad_0044[4]; //0x0044
	__m128* Torque; //0x0048
	float SpeedLimiter; //0x0050
	float SpeedLimiterNOS; //0x0054
	float SpeedLimiterReverse; //0x0058
	float EngineLoadLerp; //0x005C
	float MinLoadAtTopSpeed; //0x0060
	float ZeroLoadTorqueNoise; //0x0064
	float FullLoadTorqueNoise; //0x0068
	char pad_006C[4]; //0x006C
	__m128* TorqueNoise; //0x0070
	float IgnitionSequenceLength; //0x0078
	float RevLimiterTime; //0x007C
}; //Size: 0x0080

class RaceVehicleESCConfigData
{
public:
	char pad_0000[16]; //0x0000
	char* Name; //0x0010
	__m128* ESCOverSteerAccMinAcc; //0x0018
	__m128* ESCOverSteerDecMinAcc; //0x0020
	__m128* ESCOverSteerFrontBrake; //0x0028
	__m128* ESCOverSteerRearBrake; //0x0030
	__m128* ESCUnderSteerMinAcc; //0x0038
	__m128* ESCUnderSteerFrontBrake; //0x0040
	__m128* ESCUnderSteerRearBrake; //0x0048
	__m128* ESCUnderSteerTorque; //0x0050
}; //Size: 0x0058

class RaceVehicleForcedInductionConfigData
{
public:
	char pad_0000[16]; //0x0000
	char* Name; //0x0010
	ForcedInductionType InductionType; //0x0018
	float LagTime; //0x001C
	float SpinDownTime; //0x0020
	float BoostMinThrottle; //0x0024
	float BoostAlwaysThrottle; //0x0028
	float BoostStartRpm; //0x002C
	float PeakPSI; //0x0030
	float LowBoost; //0x0034
	float HighBoost; //0x0038
	bool HasBypassVale; //0x003C
	bool HasBlowoffValve; //0x003D
}; //Size: 0x003E

class RaceVehiclePacejkaMF52ConfigData
{
public:
	char pad_0000[16]; //0x0000
	char* Name; //0x0010
	float LFZ0; //0x0018
	float LCX; //0x001C
	float LMUX; //0x0020
	float LEX; //0x0024
	float LKX; //0x0028
	float LHX; //0x002C
	float LVX; //0x0030
	float LCY; //0x0034
	float LMUY; //0x0038
	float LEY; //0x003C
	float LKY; //0x0040
	float LHY; //0x0044
	float LVY; //0x0048
	float LGAY; //0x004C
	float LTR; //0x0050
	float LRES; //0x0054
	float LGAZ; //0x0058
	float LXAL; //0x005C
	float LYKA; //0x0060
	float LVYKA; //0x0064
	float LS; //0x0068
	float LSGKP; //0x006C
	float LSGAL; //0x0070
	float LGYR; //0x0074
	float LMX; //0x0078
	float LMY; //0x007C
	float LGAX; //0x0080
	float PCX1; //0x0084
	float PDX1; //0x0088
	float PDX2; //0x008C
	float PDX3; //0x0090
	float PEX1; //0x0094
	float PEX2; //0x0098
	float PEX3; //0x009C
	float PEX4; //0x00A0
	float PKX1; //0x00A4
	float PKX2; //0x00A8
	float PKX3; //0x00AC
	float PHX1; //0x00B0
	float PHX2; //0x00B4
	float PVX1; //0x00B8
	float PVX2; //0x00BC
	float RBX1; //0x00C0
	float RBX2; //0x00C4
	float RCX1; //0x00C8
	float RHX1; //0x00CC
	float PTX1; //0x00D0
	float PTX2; //0x00D4
	float PTX3; //0x00D8
	float REX1; //0x00DC
	float REX2; //0x00E0
	float QSX1; //0x00E4
	float QSX2; //0x00E8
	float QSX3; //0x00EC
	float PCY1; //0x00F0
	float PDY1; //0x00F4
	float PDY2; //0x00F8
	float PDY3; //0x00FC
	float PEY1; //0x0100
	float PEY2; //0x0104
	float PEY3; //0x0108
	float PEY4; //0x010C
	float PKY1; //0x0110
	float PKY2; //0x0114
	float PKY3; //0x0118
	float PHY1; //0x011C
	float PHY2; //0x0120
	float PHY3; //0x0124
	float PVY1; //0x0128
	float PVY2; //0x012C
	float PVY3; //0x0130
	float PVY4; //0x0134
	float RBY1; //0x0138
	float RBY2; //0x013C
	float RBY3; //0x0140
	float RCY1; //0x0144
	float RHY1; //0x0148
	float RVY1; //0x014C
	float RVY2; //0x0150
	float RVY3; //0x0154
	float RVY4; //0x0158
	float RVY5; //0x015C
	float RVY6; //0x0160
	float PTY1; //0x0164
	float PTY2; //0x0168
	float REY1; //0x016C
	float REY2; //0x0170
	float RHY2; //0x0174
	float QSY1; //0x0178
	float QSY2; //0x017C
	float QBZ1; //0x0180
	float QBZ2; //0x0184
	float QBZ3; //0x0188
	float QBZ4; //0x018C
	float QBZ5; //0x0190
	float QBZ9; //0x0194
	float QBZ10; //0x0198
	float QCZ1; //0x019C
	float QDZ1; //0x01A0
	float QDZ2; //0x01A4
	float QDZ3; //0x01A8
	float QDZ4; //0x01AC
	float QDZ6; //0x01B0
	float QDZ7; //0x01B4
	float QDZ8; //0x01B8
	float QDZ9; //0x01BC
	float QEZ1; //0x01C0
	float QEZ2; //0x01C4
	float QEZ3; //0x01C8
	float QEZ4; //0x01CC
	float QEZ5; //0x01D0
	float QHZ1; //0x01D4
	float QHZ2; //0x01D8
	float QHZ3; //0x01DC
	float QHZ4; //0x01E0
	float SSZ1; //0x01E4
	float SSZ2; //0x01E8
	float SSZ3; //0x01EC
	float SSZ4; //0x01F0
	float QTZ1; //0x01F4
	float MBELT; //0x01F8
	float UnloadedRadius; //0x01FC
	float NominalLoad; //0x0200
	float StaticGrip; //0x0204
}; //Size: 0x0208

class RaceVehicleSteeringConfigData
{
public:
	char pad_0000[16]; //0x0000
	char* Name; //0x0010
	__m128* BrakeInputRemap; //0x0018
	__m128* ThrottleMappingFirstGear; //0x0020
	__m128* ThrottleMappingSecondGear; //0x0028
	__m128* ThrottleMappingThirdAndAbove; //0x0030
	__m128* SteeringVsAngularVelocity; //0x0038
	__m128* SteeringVsSlipAngle; //0x0040
	__m128* SteerRate; //0x0048
	__m128* GentleInputSteerRate; //0x0050
	__m128* CounterSteerRate; //0x0058
	__m128* CenterSteerRate; //0x0060
	float ReverseSteerRate; //0x0068
	float ReverseCounterSteerRate; //0x006C
	float ReverseCenterSteerRate; //0x0070
	char pad_0074[4]; //0x0074
	__m128* SteeringRangeOffThrottle; //0x0078
	__m128* SteeringRangeOnThrottle; //0x0080
	__m128* ReverseSteeringRange; //0x0088
	float MaxSteeringAngle; //0x0090
	float MaxAngleSteeringAlignEffect; //0x0094
	float MaxAngleCounterSteerAlignEffect; //0x0098
	float DriftAngleForCounterSteerRate; //0x009C
	float DriftCounterSteerRate; //0x00A0
	float AligningTorqueEffect; //0x00A4
	float AligningTorqueEffectInDrift; //0x00A8
	float TimeThrottlingAfterDownShift; //0x00AC
	float AmountThrottlingAfterDownShift; //0x00B0
	float MinSpeedForThrottlingAfterDownShift; //0x00B4
}; //Size: 0x00B8

class RaceVehicleSteeringWheelConfigData
{
public:
	char pad_0000[16]; //0x0000
	char* Name; //0x0010
	__m128* SteeringWheelRemap; //0x0018
	__m128* SteerCentreVsSlipAngle; //0x0020
	__m128* SteerCentreVsSpeed; //0x0028
	__m128* SteerDampVsSpeed; //0x0030
	__m128* SteerWheelMaxVsSlipAngle; //0x0038
	__m128* SteerWheelMaxVsSpeed; //0x0040
	__m128* SteerWheelMaxVsSpeedMulti; //0x0048
	__m128* SteeringWheelMaxFFBVSpeed; //0x0050
	__m128* SteeringWheelSurfaceMagnitudeVSpeed; //0x0058
	__m128* SteeringWheelSurfacePeriodVSpeed; //0x0060
	__m128* SteerFFBRemapping; //0x0068
	__m128* SteerFFBVsSlipAngle; //0x0070
	__m128* SteerFFBVsSpeed; //0x0078
	float FullBrakeThreshold; //0x0080
	float NoBrakeThreshold; //0x0084
	float BrakeStabTime; //0x0088
}; //Size: 0x008C

class RaceVehicleSuspensionConfigData
{
public:
	char pad_0000[16]; //0x0000
	char* Name; //0x0010
	__m128* CamberFront; //0x0018
	__m128* CamberRear; //0x0020
	__m128* Caster; //0x0028
	float RideHeightSpecsFront; //0x0030
	float RideHeightSpecsRear; //0x0034
	float ShockBlowOut; //0x0038
	float ShockDigressionFront; //0x003C
	float ShockDigressionRear; //0x0040
	char pad_0044[4]; //0x0044
	__m128* ShockExtSpecsFront; //0x0048
	__m128* ShockExtSpecsRear; //0x0050
	__m128* ShockSpecsFront; //0x0058
	__m128* ShockSpecsRear; //0x0060
	float ShockValvingFront; //0x0068
	float ShockValvingRear; //0x006C
	float SpringProgressionFront; //0x0070
	float SpringProgressionRear; //0x0074
	float SpringSpecsFront; //0x0078
	float SpringSpecsRear; //0x007C
	__m128* SwaySpecsFront; //0x0080
	__m128* SwaySpecsRear; //0x0088
	__m128* ToeFront; //0x0090
	__m128* ToeRear; //0x0098
	float TravelSpecsFront; //0x00A0
	float TravelSpecsRear; //0x00A4
}; //Size: 0x00A8

class RaceVehicleTCSConfigData
{
public:
	char pad_0000[16]; //0x0000
	char* Name; //0x0010
	__m128* TractionControlScalarVsSlipAngle; //0x0018
	__m128* TractionControlScalarVsSpeed; //0x0020
}; //Size: 0x0028

class RaceVehicleTireConfigData
{
public:
	char pad_0000[16]; //0x0000
	char* Name; //0x0010
	TireModelType TireModelType; //0x0018
	char pad_001C[4]; //0x001C
	RaceVehiclePacejkaMF52ConfigData* PacejkaTireDataFront; //0x0020
	RaceVehiclePacejkaMF52ConfigData* PacejkaTireDataRear; //0x0028
	RaceVehicleAnalyticalTireConfigData* AnalyticalTireDataFront; //0x0030
	RaceVehicleAnalyticalTireConfigData* AnalyticalTireDataRear; //0x0038
	float TireInertiaFront; //0x0040
	float TireInertiaRear; //0x0044
	float DiameterFront; //0x0048
	float DiameterRear; //0x004C
	float RimDiameterFront; //0x0050
	float RimDiameterRear; //0x0054
	float SectionWidthFront; //0x0058
	float SectionWidthRear; //0x005C
	float StaticGripOffThrottle; //0x0060
	float BurnoutTendency; //0x0064
	__m128* RollingResistanceScalarVsSpeed; //0x0068
	class RaceVehicleTireSkidMarkConfigData* SkidMarkDataFront; //0x0070
	class RaceVehicleTireSkidMarkConfigData* SkidMarkDataRear; //0x0078
}; //Size: 0x0080

class RaceVehicleTireSkidMarkConfigData
{
public:
	char pad_0000[16]; //0x0000
	char* Name; //0x0010
	float SkidLongSlipMinIntensity; //0x0018
	float SkidLongSlipRatioThreshold; //0x001C
	float BrakeSkidLongSlipMinIntensity; //0x0020
	float BrakeSkidLongSlipRatioThreshold; //0x0024
	float SkidLatSlipMinIntensity; //0x0028
	float SkidLatSlipAngleThreshold; //0x002C
}; //Size: 0x0030

class RaceVehicleTransmissionConfigData
{
public:
	char pad_0000[16]; //0x0000
	char* Name; //0x0010
	float TransmissionInertia; //0x0018
	float ClutchSlip; //0x001C
	float DifferentialFront; //0x0020
	float DifferentialRear; //0x0024
	float DifferentialCenter; //0x0028
	float DirtTorqueSplit; //0x002C
	__m128* DownShiftClutch; //0x0030
	__m128* DownShiftSlopeScalar; //0x0038
	__m128* DownShiftThrottle; //0x0040
	float DriveShaftTorqueEffect; //0x0048
	float DriveShaftTorqueMax; //0x004C
	float ShiftDownBrakeDynamicFactor; //0x0050
	float FinalGear; //0x0054
	float TopGear; //0x0058
	char pad_005C[4]; //0x005C
	float* GearEfficiency; //0x0060
	float* GearRatio; //0x0068
	float* GearUpRPM; //0x0070
	float* GearDownRPM; //0x0078
	float* GearUpSpeed; //0x0080
	GearData* Gears; //0x0088
	float GearChangeTime; //0x0090
	char pad_0094[4]; //0x0094
	GearData* CruisingGears; //0x0098
	float GearChangeTimeCruising; //0x00A0
	float CruisingGearboxRedlineScalar; //0x00A4
	float MinTireTractionToShiftUp; //0x00A8
	float MinTireTractionToShiftUpFirstGear; //0x00AC
	float ReverseTorqueMultiplier; //0x00B0
	float TorqueSplit; //0x00B4
	float TorqueSplitInDrift; //0x00B8
	char pad_00BC[4]; //0x00BC
	__m128* UpShiftClutcb; //0x00C0
	__m128* UpShiftThrottle; //0x00C8
}; //Size: 0x00D0

class RaceVehicleTumbleConfigData
{
public:
	char pad_0000[16]; //0x0000
	char* Name; //0x0010
	float InverseMassScaleTumble; //0x0018
	float TumbleSlideSpeed; //0x001C
	float StartTumbleRadius; //0x0020
	float StartTumbleMinSpeed; //0x0024
	float StartTumbleTorqueScalarMin; //0x0028
	float StartTumbleTorqueScalarMax; //0x002C
	float StartTumbleYAngVelScalar; //0x0030
	float StartTumbleYTorqueScalar; //0x0034
	float StartTumbleMaxTargetAngularVel; //0x0038
	float StartTumbleMaxAngSpeedError; //0x003C
	float MinVelForTorque; //0x0040
	float TorqueRadius; //0x0044
	float TorqueScalarMin; //0x0048
	float TorqueScalarMax; //0x004C
	float TorqueScalarNoWheelMin; //0x0050
	float TorqueScalarNoWheelMax; //0x0054
	float AeroDragScalar; //0x0058
	float AeroDragScalarRoad; //0x005C
	float AeroDragRotScalar; //0x0060
	float AeroDragRotScalarRoad; //0x0064
	float NonZRotDamping; //0x0068
	float COGBlendTime; //0x006C
	float TumbleAngVel; //0x0070
	float SpeedToStayTumbling; //0x0074
	float TumbleCosAngle; //0x0078
	float TumbleMaxZTorque; //0x007C
	float TumbleRestitution; //0x0080
	float TumbleFriction; //0x0084
	float TumbleFrictionRoof; //0x0088
	float FrictionTumblingRoofMaxAngVel; //0x008C
	float TumbleTorqueSpreadAngle; //0x0090
	float TumbleCollisionNormalSpreadAngle; //0x0094
}; //Size: 0x0098

class RaceVehicleXCarConfigData
{
public:
	char pad_0000[16]; //0x0000
	char* Name; //0x0010
	float WheelCamberBottomFront; //0x0018
	float WheelCamberBottomRear; //0x001C
	float WheelCamberTopFront; //0x0020
	float WheelCamberTopRear; //0x0024
	float WheelCasterFront; //0x0028
	float WheelLowerLimitFront; //0x002C
	float WheelLowerLimitRear; //0x0030
	float WheelUpperLimitFront; //0x0034
	float WheelUpperLimitRear; //0x0038
	float WheelXOffsetBottomFront; //0x003C
	float WheelXOffsetBottomRear; //0x0040
	float WheelXOffsetTopFront; //0x0044
	float WheelXOffsetTopRear; //0x0048
	float WheelPivotPointXOffsetFront; //0x004C
	float WheelPivotPointXOffsetRear; //0x0050
	float XCarRideHeightFront; //0x0054
	float XCarRideHeightRear; //0x0058
	float XCarToeAngleFront; //0x005C
	float XCarToeAngleRear; //0x0060
	char pad_0064[4]; //0x0064
	__m128* XCarBodyYOffsetVSpeed; //0x0068
	float XCarMultiplier; //0x0070
	float XCarYOffset; //0x0074
	float BodyRollCenterHeight; //0x0078
	float ScalarX; //0x007C
	float ScalarY; //0x0080
	float ScalarZ; //0x0084
	float WheelSpringCoefficientFront; //0x0088
	float WheelSpringCoefficientRear; //0x008C
	float WheelSpringDampingRatioFront; //0x0090
	float WheelSpringDampingRatioRear; //0x0094
	float WheelSpringMassFront; //0x0098
	float WheelSpringMassRear; //0x009C
	float WheelSpringLowerLimitFront; //0x00A0
	float WheelSpringLowerLimitRear; //0x00A4
	float WheelSpringUpperLimitFront; //0x00A8
	float WheelSpringUpperLimitRear; //0x00AC
	float PitchForceMultiplier; //0x00B0
	float ForwardForceMultiplier; //0x00B4
	float RollForceMultiplier; //0x00B8
	float SideForceMultiplier; //0x00BC
}; //Size: 0x00C0