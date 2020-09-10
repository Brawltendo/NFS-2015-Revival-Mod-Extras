#pragma once
#include "VehicleComponents.h"

class PointGraph8
{
public:
	float min_x; //0x0000
	float min_y; //0x0004
	float max_x; //0x0008
	float max_y; //0x000C
	float x[8]; //0x0010
	float y[8]; //0x0030
}; //Size: 0x0050

class AxlePairPointGraph8 : public PointGraph8
{
public:
	class PointGraph8 Rear; //0x0050
}; //Size: 0x00A0

struct Vector2
{
	float X;
	float Y;
};

struct Matrix44
{
	__m128 xAxis;
	__m128 yAxis;
	__m128 zAxis;
	__m128 wAxis;
};

class CommonTuningState
{
public:
	float flyingCarAeroCoef_BigAir; //0x0000
	float flyingCarAeroCoef_SmallAir; //0x0004
}; //Size: 0x0008

class LateralSlipFactors
{
public:
	float copMaxLongSlipRatio; //0x0000
	float longInfluencePower; //0x0004
	float maxLongSlipRatio; //0x0008
	float maximumLongInfluence; //0x000C
	float minimumLongInfluence; //0x0010
}; //Size: 0x0014

class LongSlipFactors
{
public:
	float lateralInfluencePower; //0x0000
	float maxLateralSlipAngle; //0x0004
	float maximumLateralInfluence; //0x0008
	float minimumLateralInfluence; //0x000C
}; //Size: 0x0010

class LongGripCurveCore
{
public:
	float maxSlipForceCoefficient; //0x0000
	float maxSlipRatio; //0x0004
	float peakForceCoefficient; //0x0008
	float peakSlipRatio; //0x000C
}; //Size: 0x0010

class LateralGripCurveCore
{
public:
	float gradient; //0x0000
	float peakForceCoefficient; //0x0004
	float peakSlipAngle; //0x0008
	float slipForPeakGrip; //0x000C
}; //Size: 0x0010

class CriterionTire
{
public:
	float availablegrip; //0x0000
	float gripfactor; //0x0004
	float longitudinalbias; //0x0008
	float slidinggripfactor; //0x000C
	float wheelLoadTireResponse; //0x0010
	float torqueRatioToSlipRatio; //0x0014
	float lateralFrictionScale; //0x0018
	float longitudinalFrictionScale; //0x001C
	LateralSlipFactors brakeLateralSlipFactors; //0x0020
	LateralSlipFactors tractionLateralSlipFactors; //0x0034
	LongSlipFactors tractionLongSlipFactors; //0x0048
	LongSlipFactors brakeLongSlipFactors; //0x0058
	LongGripCurveCore brakePureLongGrip; //0x0068
	LongGripCurveCore brakeSlipLongGrip; //0x0078
	LongGripCurveCore tractionPureLongGrip; //0x0088
	LongGripCurveCore tractionSlipLongGrip; //0x0098
	LateralGripCurveCore pureLateralGrip; //0x00A8
	LateralGripCurveCore brakeSlipLateralGrip; //0x00B8
	LateralGripCurveCore tractionSlipLateralGrip; //0x00C8
	float camberAngleForMaximumGrip; //0x00D8
	float camberAngleForMinimumGrip; //0x00DC
	float camberMinimumGrip; //0x00E0
	float lateralRelaxationLength; //0x00E4
	float maximumLateralDeflection; //0x00E8
}; //Size: 0x00EC

class RaceCarAnalyticalTireData : public CriterionTire
{
public:
}; //Size: 0x00EC

class AnalyticalTireData
{
public:
	RaceCarAnalyticalTireData analyticalTireData[2]; //0x0000
	char pad_01D8[992]; //0x01D8
}; //Size: 0x05B8

class TuningState
{
public:
	__m128 tumbleLocalCOG; //0x0000
	float N00000F69; //0x0010
	float ride_height_specs[2]; //0x0014
	float travel_specs[2]; //0x001C
	float spring_specs[2]; //0x0024
	float springProgression[2]; //0x002C
	AxlePairPointGraph8 sway_specs; //0x0034
	AxlePairPointGraph8 shock_specs; //0x00D4
	AxlePairPointGraph8 shock_ext_specs; //0x0174
	float shock_valving[2]; //0x0214
	float shock_digression[2]; //0x021C
	float shockBlowout; //0x0224
	AxlePairPointGraph8 brakeSpec; //0x0228
	PointGraph8 eBrakeSpec; //0x02C8
	float brakeScalarWhenSteering[2]; //0x0318
	float brakeScalarDeadZone; //0x0320
	float tireRadius[2]; //0x0324
	float rimRadius[2]; //0x032C
	float wheelUpperLimit[2]; //0x0334
	float wheelLowerLimit[2]; //0x033C
	float wheelCamberBottom[2]; //0x0344
	float wheelCamberTop[2]; //0x034C
	float wheelCaster; //0x0354
	float wheelXOffsetBottom[2]; //0x0358
	float wheelXOffsetTop[2]; //0x0360
	float wheelPivotPointXOffset[2]; //0x0368
	float torqueSplit; //0x0370
	float torqueSplitInDrift; //0x0374
	float dirtTorqueSplit; //0x0378
	float differentialFactor[3]; //0x037C
	float burnoutTendency; //0x0388
	float dragCoefficient; //0x038C
	float nitrousDragCoefficient; //0x0390
	float aeroTopSpeed; //0x0394
	float aeroNitrousTopSpeed; //0x0398
	float aeroCenterOfGravity; //0x039C
	PointGraph8 aeroCG; //0x03A0
	PointGraph8 aeroCoefficient; //0x03F0
	float groundEffectHeightRatio; //0x0440
	float liftHeightRatio; //0x0444
	float aeroCoefficientMaxGroundEffect; //0x0448
	float aeroCoefficientMaxLift; //0x044C
	float aeroCoefficientOffThrottle; //0x0450
	char pad_0454[80]; //0x0454
	float wheelBase; //0x04A4
	float trackWidth[2]; //0x04A8
	float sectionWidth[2]; //0x04B0
	float frontBrakeOffset; //0x04B8
	float rearBrakeOffset; //0x04BC
	float frontAxle; //0x04C0
	PointGraph8 rollCenter; //0x04C4
	AxlePairPointGraph8 toe; //0x0514
	AxlePairPointGraph8 camber; //0x05B4
	PointGraph8 caster; //0x0654
	float NOSGripIncrease; //0x06A4
	float staticGripOffThrottle; //0x06A8
	float driveshaftTorqueEffect; //0x06AC
	float driveshaftTorqueMax; //0x06B0
	PointGraph8 frontWeightBias; //0x06B4
	float mass; //0x0704
	float collisionInvMassScale; //0x0708
	float powerToFrontWheels; //0x070C
	float powerToRearWheels; //0x0710
	float idle; //0x0714
	float redLine; //0x0718
	float maxRPM; //0x071C
	float flyWheelMass; //0x0720
	float frontWheelRadius; //0x0724
	float rearWheelRadius; //0x0728
	PointGraph8 torqueCurve; //0x072C
	PointGraph8 torqueFrictionCurve; //0x077C
	PointGraph8 engineBrakingVsGear; //0x07CC
	float unloadedEngineFriction; //0x081C
	float revLimiterTime; //0x0820
	float gearRatioValues[10]; //0x0824
	__int32 numberOfGearRatiosValues; //0x084C
	float gearEfficiencyValues[10]; //0x0850
	__int32 numberOfGearEfficiencyValues; //0x0878
	float peakTorqueRPM; //0x087C
	float clutchSlip; //0x0880
	float finalGear; //0x0884
	float antiBogMaxMultiplier; //0x0888
	float antiBogRampTime; //0x088C
	float speedLimiter; //0x0890
	float speedLimiterNos; //0x0894
	float reverseSpeedLimiter; //0x0898
	float reverseTorqueMultiplier; //0x089C
	PointGraph8 tractionControlScalarVsSlipAngle; //0x08A0
	PointGraph8 tractionControlScalarVsSpeed; //0x08F0
	GearID topGear; //0x0940
	__int32 numberOfShiftUpRPMValues; //0x0944
	float shiftUpRPMValues[10]; //0x0948
	__int32 numberOfShiftDownRPMValues; //0x0970
	float shiftDownRPMValues[10]; //0x0974
	__int32 numberOfShiftUpSpeedValues; //0x099C
	float shiftUpSpeedValues[10]; //0x09A0
	float minTireTractionToShiftUpFirstGear; //0x09C8
	float minTireTractionToShiftUp; //0x09CC
	float shiftDownBrakeDynamicFactor; //0x09D0
	PointGraph8 UpShiftThrottle; //0x09D4
	PointGraph8 UpShiftClutch; //0x0A24
	PointGraph8 DownShiftThrottle; //0x0A74
	PointGraph8 DownShiftClutch; //0x0AC4
	PointGraph8 DownShiftSlopeScalar; //0x0B14
	float forcedIndHighBoost; //0x0B64
	float forcedIndLowBoost; //0x0B68
	eInductionType forcedInductionType; //0x0B6C
	float forcedIndBoostStartRPM; //0x0B70
	float forcedIndBoostAlwaysThrottle; //0x0B74
	float forcedIndBoostMinThrottle; //0x0B78
	float forcedIndPeakPSI; //0x0B7C
	bool forcedIndHasBypassValve; //0x0B80
	bool forcedIndHasBlowoffValve; //0x0B81
	char pad_0B82[2]; //0x0B82
	float forcedIndLagTime; //0x0B84
	float forcedIndSpinDownTime; //0x0B88
	PointGraph8 throttleMappingFirstGear; //0x0B8C
	PointGraph8 throttleMappingSecondGear; //0x0BDC
	PointGraph8 throttleMappingThirdAndAbove; //0x0C2C
	PointGraph8 brakeInputRemap; //0x0C7C
	float timeThrottlingAfterDownShift; //0x0CCC
	float amountThrottlingAfterDownShift; //0x0CD0
	float minSpeedForThrottlingAfterDownShift; //0x0CD4
	PointGraph8 steeringWheelSurfaceMagnitudeVSpeed; //0x0CD8
	PointGraph8 steeringWheelSurfacePeriodVSpeed; //0x0D28
	PointGraph8 steeringWheelMaxFFBVSpeed; //0x0D78
	PointGraph8 steerWheelMaxVsSpeed; //0x0DC8
	PointGraph8 steerWheelMaxVsSpeedMulti; //0x0E18
	PointGraph8 steerWheelMaxVsSlipAngle; //0x0E68
	PointGraph8 steerFFBVsSlipAngle; //0x0EB8
	PointGraph8 steerFFBVsSpeed; //0x0F08
	PointGraph8 steerCentreVsSpeed; //0x0F58
	PointGraph8 steerCentreVsSlipAngle; //0x0FA8
	PointGraph8 steerDampVsSpeed; //0x0FF8
	PointGraph8 steerFFBRemapping; //0x1048
	float slipAngleToEnterDrift; //0x1098
	float unk1; //0x109C
	__m128 dimension; //0x10A0
	float extraFishTailTorque; //0x10B0
	float ABSTorqueRatioLowerBound; //0x10B4
	float ABSTorqueRatioUpperBound; //0x10B8
	float ABSSlipRatioSystemTargetSR; //0x10BC
	bool ABSSlipRatioSystemEnabled; //0x10C0
	char pad_10C1[3]; //0x10C1
	PointGraph8 ABSTorqueRatioScalarVsSpeed; //0x10C4
	PointGraph8 ESCOverSteerAccMinAcc; //0x1114
	PointGraph8 ESCOverSteerDecMinAcc; //0x1164
	PointGraph8 ESCUnderSteerMinAcc; //0x11B4
	PointGraph8 ESCOverSteerFrontBrake; //0x1204
	PointGraph8 ESCOverSteerRearBrake; //0x1254
	PointGraph8 ESCUnderSteerFrontBrake; //0x12A4
	PointGraph8 ESCUnderSteerRearBrake; //0x12F4
	PointGraph8 ESCUnderSteerTorque; //0x1344
	PointGraph8 xCarBodyYOffsetVSpeed; //0x1394
	float xCarRideHeight[2]; //0x13E4
	float xCarToeAngle[2]; //0x13EC
	float xCarMultiplier; //0x13F4
	float xCarYOffset; //0x13F8
	float downforce; //0x13FC
	float downForceOffset; //0x1400
	float downForceOffsetUnderBraking; //0x1404
	float downForceOffsetInDrift; //0x1408
	CommonTuningState commonTuning; //0x140C
	TireModelType tireModelType; //0x1414
	AnalyticalTireData analyticalTireData; //0x1418
	PointGraph8 steerRate; //0x19D0
	PointGraph8 gentleInputSteerRate; //0x1A20
	PointGraph8 counterSteerRate; //0x1A70
	PointGraph8 centerSteerRate; //0x1AC0
	float reverseSteerRate; //0x1B10
	float reverseCounterSteerRate; //0x1B14
	float reverseCenterSteerRate; //0x1B18
	PointGraph8 steeringRangeOffThrottle; //0x1B1C
	PointGraph8 steeringRangeOnThrottle; //0x1B6C
	PointGraph8 steeringRangeInReverse; //0x1BBC
	float maxSteeringAngle; //0x1C0C
	float maxAngleSteeringAlignEffect; //0x1C10
	float maxAngleCounterSteerAlignEffect; //0x1C14
	float driftAngleForCounterSteerRate; //0x1C18
	float driftCountersteerRate; //0x1C1C
	float frontTireInertia; //0x1C20
	float rearTireInertia; //0x1C24
	float gearChangeTime; //0x1C28
	float gearChangeTimeCruising; //0x1C2C
	float engineLoadLerp; //0x1C30
	float minLoadAtTopSpeed; //0x1C34
	float zeroLoadTorqueNoise; //0x1C38
	float fullLoadTorqueNoise; //0x1C3C
	PointGraph8 torqueNoise; //0x1C40
	PointGraph8 rollingResistanceVsSpeed; //0x1C90
}; //Size: 0x1CE0

class InputState
{
public:
	__m128 localVelocity; //0x0000
	__m128 linearVelocity; //0x0010
	__m128 angularVelocity; //0x0020
	__m128 forwardVector; //0x0030
	bool requestShift; //0x0040
	char pad_0041[3]; //0x0041
	GearID requestShift_Gear; //0x0044
	bool requestMatchSpeed; //0x0048
	char pad_0049[3]; //0x0049
	float requestMatchSpeed_Speed; //0x004C
	bool requestForceCompression[4]; //0x0050
	float requestForceCompression_CompressionValue[4]; //0x0054
	bool requestClutchKick; //0x0064
	char pad_0065[3]; //0x0065
	float steeringSpeedScalar; //0x0068
	float gameBreakerMaxSteer; //0x006C
	float steerSpeedMultiplier; //0x0070
	SteeringType steeringType; //0x0074
	bool isStaging; //0x0078
	char pad_0079[3]; //0x0079
	float speed; //0x007C
	float prevFrameSpeed; //0x0080
	float forwardSpeed; //0x0084
	unsigned __int32 numberOfSubSteps; //0x0088
	float inputGas; //0x008C
	float inputSteering; //0x0090
	float inputBrake; //0x0094
	float inputEBrake; //0x0098
	float timeFullGasInput; //0x009C
	bool manualGearUp; //0x00A0
	bool manualGearDown; //0x00A1
	bool gameWasPaused; //0x00A2
	char pad_00A3[1]; //0x00A3
	float mass; //0x00A4
	char pad_00A8[8]; //0x00A8
	Matrix44 matrix; //0x00B0
	__int16 driverStyle; //0x00F0
	bool isHumanPlayer; //0x00F2
	bool isTraffic; //0x00F3
	float slipangle; //0x00F4
	float slipangleCorrect; //0x00F8
	bool isAutomaticShifting; //0x00FC
	bool isABSEnabled; //0x00FD
	bool isTCSEnabled; //0x00FE
	bool engineBlown; //0x00FF
	__int32 blownTires; //0x0100
	bool nosEngaged; //0x0104
	char pad_0105[3]; //0x0105
	float nosTorqueMultiplier; //0x0108
	bool onValidWorldFace[4]; //0x010C
	__m128 groundNormal; //0x0110
	bool isModelling; //0x0120
	char pad_0121[3]; //0x0121
	float gravity; //0x0124
	bool onMovingGround; //0x0128
	char pad_0129[3]; //0x0129
	float gameBreaker; //0x012C
	float groundEffect; //0x0130
	__int32 flags; //0x0134
	bool isAIControlled; //0x0138
	bool isAnimationControlled; //0x0139
	char pad_013A[2]; //0x013A
	float reverseJTurnForceScale; //0x013C
	SleepState sleepState; //0x0140
	float speedAwayFromSurface; //0x0144
	char pad_0148[8]; //0x0148
	__m128 world_wheelVelocity[4]; //0x0150
	float wheelMountHeightAboveGround[4]; //0x0190
	__m128 groundNormalUnderWheel[4]; //0x01A0
	__m128 groundVelocityAtWheel[4]; //0x01E0
	float surfaceDriveGrip[4]; //0x0220
	float surfaceRollingResistance[4]; //0x0230
	float surfaceVelocityDrag[4]; //0x0240
	float surfaceRugosityAmplitude[4]; //0x0250
	float surfaceRugosityFrequency[4]; //0x0260
	float lateralGripScalar[4]; //0x0270
	float longitudinalGripScalar[4]; //0x0280
	float longGripThrottleScalar[4]; //0x0290
	float longGripBrakeScalar[4]; //0x02A0
	bool naturalAirExpected; //0x02B0
	bool alignToRoad; //0x02B1
	char pad_02B2[2]; //0x02B2
	float averageDriveGrip; //0x02B4
	float drafting; //0x02B8
	float draftingSpeed; //0x02BC
	float camber[4]; //0x02C0
	bool isTumbling; //0x02D0
	bool isDrifting; //0x02D1
	bool counterSteeringInDrift; //0x02D2
	bool doNotUpdateMaxSteeringAngles; //0x02D3
	float aligningTorqueEffect; //0x02D4
	float aligningTorqueCenter; //0x02D8
	float aligningTorqueStrength; //0x02DC
	bool overrideSteering; //0x02E0
	char pad_02E1[3]; //0x02E1
	float overrideSteeringRadians; //0x02E4
	float overrideSteeringScale; //0x02E8
	float driftAngle; //0x02EC
	float steeringLimitLeft; //0x02F0
	float steeringLimitRight; //0x02F4
	char pad_02F8[8]; //0x02F8
	__m128 worldInverseInertiaTensorX; //0x0300
	__m128 worldInverseInertiaTensorY; //0x0310
	__m128 worldInverseInertiaTensorZ; //0x0320
	float requestClutch; //0x0330
	bool isShiftingUp; //0x0334
	bool isShiftingDown; //0x0335
	char pad_0336[2]; //0x0336
	float landingStabilityScale; //0x0338
	float burnoutScale; //0x033C
}; //Size: 0x0340

class OutputState
{
public:
	__m128 forceAppliedToCar; //0x0000
	__m128 torqueAppliedToCar; //0x0010
	__m128 linearImpulse; //0x0020
	__m128 angularImpulse; //0x0030
	__m128 aeroSideForce; //0x0040
	__m128 aeroDownForce; //0x0050
	__m128 aeroDragForce; //0x0060
	__m128 aeroDownCenter; //0x0070
	__m128 aeroSideCenter; //0x0080
	__m128 aeroDragCenter; //0x0090
	__m128 aeroDragTorque; //0x00A0
	__m128 localCOG; //0x00B0
	__m128 tireLateralNormal[4]; //0x00C0
	__m128 tireSteeringNormal[4]; //0x0100
	__m128 world_wheelTirePatchPosition[4]; //0x0140
	__m128 local_wheelTirePatchPosition[4]; //0x0180
	__m128 local_wheelMountPosition[4]; //0x01C0
	float local_steeringWheelAngles[2]; //0x0200
	float suspensionCompression[4]; //0x0208
	bool suspensionIsAtRest; //0x0218
	char pad_0219[3]; //0x0219
	float speedometer; //0x021C
	float maxSpeedometer; //0x0220
	bool vehicleIsAsleep; //0x0224
	char pad_0225[3]; //0x0225
	float wheelieAngle; //0x0228
	bool staticResetConditionMet; //0x022C
	char pad_022D[3]; //0x022D
	float tireTraction[4]; //0x0230
	float tireSlipRatio[4]; //0x0240
	float tireLongSlip[4]; //0x0250
	float tireSlipAngle[4]; //0x0260
	float tireRadius[4]; //0x0270
	float tireLateralVelocity[4]; //0x0280
	float tireAngularVelocity[4]; //0x0290
	float tireLateralForce[4]; //0x02A0
	bool isABSOn[4]; //0x02B0
	float suspensionDigression[4]; //0x02B4
	float suspensionLoad[4]; //0x02C4
	__int32 numberOfWheelsOnGround; //0x02D4
	float steeringRangeLeft; //0x02D8
	float steeringRangeRight; //0x02DC
	float steeringSpeed; //0x02E0
	float ffbTorque; //0x02E4
	float maxHorsePower; //0x02E8
	float maxHorsePowerRPM; //0x02EC
	float minHorsePower; //0x02F0
	float currentHorsePower; //0x02F4
	float engineRpm; //0x02F8
	float driveShaftRpm; //0x02FC
	float peakTorque; //0x0300
	float peakTorqueRPM; //0x0304
	float throttle; //0x0308
	float clutch; //0x030C
	ClutchState clutchState; //0x0310
	float torqueRatio; //0x0314
	float clutchGrind; //0x0318
	float forcedIndSpool; //0x031C
	float forcedIndCurrentPSI; //0x0320
	float forcedIndRelativeTorqueGain; //0x0324
	float forcedIndByPassPosition; //0x0328
	bool forcedIndIsBlowoffOpened; //0x032C
	bool forcedIndIsWestgateOpened; //0x032D
	bool isGearChanging; //0x032E
	char pad_032F[1]; //0x032F
	ShiftStatus shiftStatus; //0x0330
	ShiftPotential shiftPotential; //0x0334
	float totalShiftTime; //0x0338
	float timeUntiTransition; //0x033C
	float shiftUpRpm; //0x0340
	float shiftDownRpm; //0x0344
	GearID gear; //0x0348
	float torqueSplit; //0x034C
	float engineTorque; //0x0350
	float driveShaftTorque; //0x0354
	float draftGauge; //0x0358
	float draftSlingshot; //0x035C
	bool draftHasFilledGauge; //0x0360
	char pad_0361[3]; //0x0361
	float unk; //0x0364
	float wheelSteeringAngleRadians; //0x0368
	float wheelVisualSteeringAngleRadians; //0x036C
	float steeringScale; //0x0370
	float revLimiterTimeLeft; //0x0374
}; //Size: 0x0378

class VehicleInput
{
public:
	VehicleInputData* inputData; //0x0000
	Vector2 steeringInput; //0x0008
	float unk; //0x00010
	char pad_0014[16]; //0x0014
	Vector2 throttleInput; //0x0024
	Vector2 brakeInput; //0x002C
	float handbrakeInput; //0x0034
	char pad_0038[80]; //0x0038
}; //Size: 0x0088

class NFSVehicle
{
public:
	RaceVehicleConfigData* vehicleConfig; //0x0000
	char pad_0008[16]; //0x0008
	VehicleInput* vehicleInput; //0x0018
	class PointerToWheelData* pWheelData; //0x0020
	char pad_0028[128]; //0x0028
	class N00003CA7* N00002FB5; //0x00A8
	char pad_00B0[16]; //0x00B0
	class DynamicPhysicsEntity* dynamicPhysEnt; //0x00C0
	char pad_00C8[32]; //0x00C8
	class SimplePhysicsEntity* simplePhysEnt; //0x00E8
	char pad_00F0[40]; //0x00F0
	class PointerToRaceCar* pointerToRaceCar; //0x0118
	char pad_0120[592]; //0x0120
	class VehicleCollisionCallbacks* vehicleCollisionCallbacks; //0x0370
	char pad_0378[120]; //0x0378
	float mass; //0x03F0
	char pad_03F4[44]; //0x03F4
	float speedMps; //0x0420
	char pad_0424[68]; //0x0424
	class RaceCar* raceCar; //0x0468
	InputState inputState; //0x0470
	OutputState outputState; //0x07B0
	char pad_0B28[400]; //0x0B28
	bool isABSEnabled; //0x0CB8
	bool isTCSEnabled; //0x0CB9
	char pad_0CBA[82]; //0x0CBA
	NFSVehicleState vehicleState; //0x0D0C
	char pad_0D10[360]; //0x0D10
	float deltaTime; //0x0E78
	float motionScalar; //0x0E7C
	char pad_0E80[224]; //0x0E80
	float landingStabilityScale; //0x0F60
	float timeInAir; //0x0F64
	float timeSinceLastAirTime; //0x0F68
	char pad_0F6C[320]; //0x0F6C
	PointGraph8 N00003DC1; //0x10AC
	PointGraph8 N00003DCB; //0x10FC
	char pad_114C[4]; //0x114C
	class PerformanceModificationComponent* perfModComponent; //0x1150
	char pad_1158[48]; //0x1158
	DriverComponent* driverComponent; //0x1188
	float N0000EAA7; //0x1190
	float N0000EFB8; //0x1194
	HandbrakeComponent* handbrakeComponent; //0x1198
	DonutComponent* donutComponent; //0x11A0
	DriftComponent* driftComponent; //0x11A8
	DriftParams* driftParams; //0x11B0
	class RaceCarPhysicsObject* raceCarPhysObj; //0x11B8
	SteeringComponent* steeringComponent; //0x11C0
	RaceRigidBody* raceRigidBody; //0x11C8
	char pad_11D0[356]; //0x11D0
	class N0000F5A0* N0000E9FC; //0x1334
	char pad_133C[2516]; //0x133C
}; //Size: 0x1D10

class RaceRigidBody
{
public:
	NFSVehicle* nfsVehicle; //0x0000
	char pad_0008[128]; //0x0008
}; //Size: 0x0088