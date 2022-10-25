#pragma once
#include <stdint.h>

#if !USE_REVIVAL_COMPONENT
namespace fb
{
#endif // !USE_REVIVAL_COMPONENT

enum RaceVehicleModificationType
{
    ModificationType_Scalar = 0,
    ModificationType_Addition = 1,
    ModificationType_Override = 2,
};

enum RaceVehicleAttributeToModify
{
    ATM_EngineTorque = 0,
    ATM_DriftYawTorque = 1,
    ATM_FrontDrag = 2,
    ATM_XCarMultiplier = 3,
    ATM_SuspensionStiffness_Front = 4,
    ATM_SuspensionStiffness_Rear = 5,
    ATM_TorqueSplit = 6,
    ATM_DiameterFront = 7,
    ATM_DiameterRear = 8,
    ATM_RimDiameterFront = 9,
    ATM_RimDiameterRear = 10,
    ATM_RevLimiterTime = 11,
    ATM_Mass = 12,
    ATM_Strength = 13,
    ATM_GearChangeTime = 14,
    ATM_DownForceEffectOnBody = 15,
    ATM_FrontFinalLongGripForZeroSlip = 16,
    ATM_FrontSlipInfluenceOnLongGrip = 17,
    ATM_RearFinalLongGripForZeroSlip = 18,
    ATM_RearSlipInfluenceOnLongGrip = 19,
    ATM_RollAngleLimit = 20,
    ATM_MaintainSpeedInDriftAmount = 21,
    ATM_MinTireTractionToShiftUp = 22,
    ATM_MinTireTractionToShiftUpFirstGear = 23,
    ATM_GravityMultiplier = 24,
    ATM_Downforce = 25,
    ATM_MinimumAngleForDrift = 26,
    ATM_SideForceMagnitude = 27,
    ATM_FrontDragSpeedThreshold = 28,
    ATM_NitrousFrontDragSpeedThreshold = 29,
    ATM_FinalDriveRatio = 30,
    ATM_GearDownRPM = 31,
    ATM_GearUpSpeed = 32,
    ATM_GearEfficiency0 = 33,
    ATM_GearEfficiency1 = 34,
    ATM_GearEfficiency2 = 35,
    ATM_GearEfficiency3 = 36,
    ATM_GearEfficiency4 = 37,
    ATM_GearEfficiency5 = 38,
    ATM_GearEfficiency6 = 39,
    ATM_GearEfficiency7 = 40,
    ATM_GearEfficiency8 = 41,
    ATM_GearEfficiency9 = 42,
    ATM_GearEfficiency10 = 43,
    ATM_LongitudinalFrictionScaleFront = 44,
    ATM_LongitudinalFrictionScaleRear = 45,
    ATM_SpeedLimiter = 46,
    ATM_SpeedLimiterNOS = 47,
    ATM_BurnoutTendency = 48,
    ATM_GentleInputSteerRate = 49,
    ATM_VelocityDragMultiplier = 50,
    ATM_SlipAngleForMinDragMultiplier = 51,
    ATM_SlipAngleForMaxDragMultiplier = 52,
    ATM_MinDragMultiplier = 53,
    ATM_MaxDragMultiplier = 54,
    ATM_TopGear = 55,
    ATM_LateralFrictionScaleFront = 56,
    ATM_LateralFrictionScaleRear = 57,
    ATM_FrontBrakeTorqueVsSpeed = 58,
    ATM_RearBrakeTorqueVsSpeed = 59,
    ATM_SteerRate = 60,
    ATM_XCarWheelCamberBottomFront = 61,
    ATM_XCarWheelCamberBottomRear = 62,
    ATM_XCarWheelCamberTopFront = 63,
    ATM_XCarWheelCamberTopRear = 64,
    ATM_XCarWheelCasterFront = 65,
    ATM_GearRatio0 = 66,
    ATM_GearRatio1 = 67,
    ATM_GearRatio2 = 68,
    ATM_GearRatio3 = 69,
    ATM_GearRatio4 = 70,
    ATM_GearRatio5 = 71,
    ATM_GearRatio6 = 72,
    ATM_GearRatio7 = 73,
    ATM_GearRatio8 = 74,
    ATM_GearRatio9 = 75,
    ATM_GearRatio10 = 76,
    ATM_DownforceOffsetUnderBraking = 77,
    ATM_HandbrakeTorqueVsSpeed = 78,
    ATM_CounterSteerRate = 79,
    ATM_CenterSteerRate = 80,
    ATM_SteeringRangeOffThrottle = 81,
    ATM_SteeringRangeOnThrottle = 82,
    ATM_ReverseSteeringRange = 83,
    ATM_DriftCounterSteerRate = 84,
    ATM_MaxSteeringAngle = 85,
    ATM_MinHandbrakeTime = 86,
    ATM_TrackWidthFront = 87,
    ATM_TrackWidthRear = 88,
    ATM_CamberFront = 89,
    ATM_CamberRear = 90,
    ATM_RideHeightSpecsFront = 91,
    ATM_RideHeightSpecsRear = 92,
    ATM_ToeFront = 93,
    ATM_ToeRear = 94,
    ATM_ReduceForwardSpeedAmount = 95,
    ATM_SlipAngleToStartBlendingDownDriftScale = 96,
    ATM_SlipAngleForZeroDriftScale = 97,
    ATM_DriftScaleFromBraking = 98,
    ATM_SlipAngleToEnterWhenBraking = 99,
    ATM_SlipAngleToEnterWhenHandbraking = 100,
    ATM_SlipAngleToEnterWhenBrakeStab = 101,
    ATM_SlipAngleToEnterWhenScandinavianFlick = 102,
    ATM_StartingDriftScaleAfterScandinavianFlick = 103,
    ATM_DriftAngleToExitDrift = 104,
    ATM_DriftAngularDamping = 105,
    ATM_DriftScaleFromHandbrake = 106,
    ATM_DriftScaleDecay = 107,
    ATM_DriftScaleFromSteering = 108,
    ATM_DriftScaleFromCounterSteering = 109,
    ATM_DriftScaleFromGasLetOff = 110,
    ATM_DriftSidewaysDamping = 111,
    ATM_DonutLongitudinalGripMin = 112,
    ATM_DonutLongitudinalGripMax = 113,
    ATM_DonutLateralGripMin = 114,
    ATM_DonutLateralGripMax = 115,
    ATM_DifferentialFront = 116,
    ATM_DifferentialCenter = 117,
    ATM_DifferentialRear = 118,
    ATM_NOS_DragCoefficient = 119,
    ATM_XCarYOffset = 120,
    ATM_XCarBodyYOffset = 121,
    ATM_XCarRideHeightFront = 122,
    ATM_XCarRideHeightRear = 123,
    ATM_XCarWheelXOffsetBottomFront = 124,
    ATM_XCarWheelXOffsetTopFront = 125,
    ATM_XCarWheelXOffsetBottomRear = 126,
    ATM_XCarWheelXOffsetTopRear = 127,
    ATM_XCarToeAngleFront = 128,
    ATM_XCarToeAngleRear = 129,
    ATM_DefaultSteering = 130,
    ATM_DriftScaleFromHandbrakeAtLowSpeed = 131,
    ATM_DriftScaleFromHandbrakeAtHighSpeed = 132,
    ATM_ExtraFishTailTorque = 133,
    ATM_GasLetOffYawTorque = 134,
    ATM_DefaultSteeringRemapping = 135,
    ATM_CounterSteeringRemapping = 136,
    ATM_XCarScalarX = 137,
    ATM_XCarScalarY = 138,
    ATM_XCarScalarZ = 139,
    ATM_DownforceOffset = 140,
    ATM_DownforceOffsetInDrift = 141,
    ATM_BackwardsExtraBrakeStrength = 142,
    ATM_LongGripForZeroSlipAnglePeakGripFront = 143,
    ATM_BrakingLongGripForZeroSlipAnglePeakGripFront = 144,
    ATM_BrakingLongGripForZeroSlipAngleFinalGripFront = 145,
    ATM_LongGripForZeroSlipAnglePeakGripRear = 146,
    ATM_BrakingLongGripForZeroSlipAnglePeakGripRear = 147,
    ATM_BrakingLongGripForZeroSlipAngleFinalGripRear = 148,
    ATM_TorqueSplitInDrift = 149,
    ATM_BrakingLateralGripForLargeSpinRatioInitialGripFront = 150,
    ATM_BrakingLateralGripForLargeSpinRatioInitialGripRear = 151,
    ATM_BrakingLateralGripForLargeSpinRatioPeakGripFront = 152,
    ATM_BrakingLateralGripForLargeSpinRatioPeakGripRear = 153,
    ATM_AligningTorqueEffectInDrift = 154,
    ATM_SlipAngleForFullDampingFade = 155,
    ATM_DriftScaleFromGasStab = 156,
    ATM_WheelPivotPointXOffsetFront = 157,
    ATM_WheelPivotPointXOffsetRear = 158,
    ATM_Max = 159
};

struct __declspec(align(4)) ModifiedValueCache
{
    float scalar;
    float modifier;
    bool useOverride;
};

struct __declspec(align(4)) PerformanceModifier
{
    RaceVehicleModificationType modificationType;
    float modifier;
};

class __declspec(align(8)) PerformanceModificationComponent
{
public:
    // Generalized function for returning the modified value of any performance attributes
    float GetModifiedValue(int attributeToModify, float unmodifiedValue);

    int64_t pad_0000[5];
    class RaceVehiclePerformanceModifierData* m_vehicleModifications;
    class RaceVehiclePerformanceModifierData* m_gearChangeTimeMods;
    int64_t pad_0038[48];
    ModifiedValueCache m_modifiedValueCache[ATM_Max];
    PerformanceModifier m_modifiers[ATM_Max];

}; static_assert(sizeof(PerformanceModificationComponent) == 0xE28, "PerformanceModificationComponent size must be 0xE28");

#if !USE_REVIVAL_COMPONENT
}
#endif // !USE_REVIVAL_COMPONENT


