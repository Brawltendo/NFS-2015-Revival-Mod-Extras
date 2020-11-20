#pragma once
#include <vector>
#include "VehicleComponents.h"

using namespace std;

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

class SteeringStaticState : public PointGraph8
{
public:
	class PointGraph8 steerWheelMaxVsSpeedMulti; //0x0050
	class PointGraph8 steerWheelMaxVsSlipAngle; //0x00A0
	class PointGraph8 GentleInputSteerRate; //0x00F0
	class PointGraph8 SteerRate; //0x0140
	class PointGraph8 CounterSteerRate; //0x0190
	class PointGraph8 CenterSteerRate; //0x01E0
	class PointGraph8 SteeringRangeOffThrottle; //0x0230
	class PointGraph8 SteeringRangeOnThrottle; //0x0280
	class PointGraph8 SteeringRangeInReverse; //0x02D0
	float ReverseSteerRate; //0x0320
	float ReverseCounterSteerRate; //0x0324
	float ReverseCenterSteerRate; //0x0328
	float MaxSteeringAngle; //0x032C
	float MaxAngleSteeringAlignEffect; //0x0330
	float MaxAngleCounterSteerAlignEffect; //0x0334
	float DriftAngleForCounterSteerRate; //0x0338
	float DriftCountersteerRate; //0x033C
	float SlipAngleToEnterDrift; //0x0340
}; //Size: 0x0344

class TransmissionStaticState
{
public:
	float differentialFactor[3]; //0x0000
	float torqueSplit; //0x000C
}; //Size: 0x0010

class StabilityManagementState
{
public:
	float speed; //0x0000
	float acceleration; //0x0004
	float time; //0x0008
	float slipAngle; //0x000C
	float yawRate; //0x0010
	float steerInput; //0x0014
	float torqueSplit; //0x0018
	float underSteerTimer; //0x001C
}; //Size: 0x0020

class TireStaticState
{
public:
	float ABSTorqueRatioLowerBound; //0x0000
	float ABSTorqueRatioUpperBound; //0x0004
	bool ABSSlipRatioSystemEnabled; //0x0008
	char pad_0009[3]; //0x0009
	float ABSSlipRatioSystemTargetSR; //0x000C
	class PointGraph8 ABSTorqueRatioScalarVsSpeed; //0x0010
	class PointGraph8 steerFFBVsSlipAngle; //0x0060
	class PointGraph8 steerFFBVsSpeed; //0x00B0
}; //Size: 0x0100

class AerodynamicsStaticState
{
public:
	__m128 carBodyDimension; //0x0000
	class PointGraph8 aeroCoefficientScalarVsSlipAngle; //0x0010
	class PointGraph8 aeroCoefficientSlipScalarVsSpeed; //0x0060
	float liftHeightRatio; //0x00B0
	float bodyRideHeightNoDownforce; //0x00B4
	float aeroCoefficientMaxGroundEffect; //0x00B8
	float aeroCoefficientMaxLift; //0x00BC
	float minHeightAtRest; //0x00C0
	float groundEffectHeightRatio; //0x00C4
	float flyingCarAeroCoefficient_SmallAir; //0x00C8
	float flyingCarAeroCoefficient_BigAir; //0x00CC
	float topSpeed; //0x00D0
	float nitrousTopSpeed; //0x00D4
	float N00001F23; //0x00D8
	char pad_00DC[4]; //0x00DC
}; //Size: 0x00E0

class TirePatchStaticState
{
public:
	char pad_0000[4]; //0x0000
	class CriterionTire criterionTire; //0x0004
	char pad_00F0[496]; //0x00F0
}; //Size: 0x02E0

class ChassisInternalState
{
public:
	__m128 mSteerL; //0x0000
	__m128 mSteerR; //0x0010
	float mPreviousSpeed; //0x0020
	bool mForcedCompression; //0x0024
	char pad_0025[3]; //0x0025
	float mSteeringTimer; //0x0028
	float mSleepTime; //0x002C
	float mBrakeAssistGrip[4]; //0x0030
	float mTraction[4]; //0x0040
	float mCamberAngle[4]; //0x0050
}; //Size: 0x0060

class CoreChassis : public TransmissionStaticState
{
public:
	class StabilityManagementState mStabilityManagementState; //0x0010
	int32_t mSuspensionStaticState; //0x0030
	class TireStaticState mTireStaticState[4]; //0x0034
	float N00000A4F; //0x0434
	float N00000A94; //0x0438
	float N00000A50; //0x043C
	class AerodynamicsStaticState mAeroStaticState; //0x0440
	class TirePatchStaticState mTirePatchStaticState[4]; //0x0520
	class ChassisInternalState mInternal; //0x10A0
	char pad_1100[16]; //0x1100
}; //Size: 0x1110

class CommonVehicleState
{
public:
	uint32_t actorId; //0x0000
}; //Size: 0x0004

class ShiftingAIStaticState : public CommonVehicleState
{
public:
	GearID topGear; //0x0004
	uint32_t numberOfShiftUpRPMValues; //0x0008
	float shiftUpRPMValues[10]; //0x000C
	uint32_t numberOfShiftDownRPMValues; //0x0034
	float shiftDownRPMValues[10]; //0x0038
	float minTireTractionToShiftUpFirstGear; //0x0060
	float minTireTractionToShiftUp; //0x0064
	float shiftDownBrakeDynamicFactor; //0x0068
	float idleRPM; //0x006C
	float maxRPM; //0x0070
	float redLineRPM; //0x0074
	float regularUpShiftTime; //0x0078
	float regularDownShiftTime; //0x007C
}; //Size: 0x0080

class ThrottleAIStaticState : public CommonVehicleState
{
public:
	class PointGraph8 throttleMappingFirstGear; //0x0004
	class PointGraph8 throttleMappingSecondGear; //0x0054
	class PointGraph8 throttleMappingThirdAndAbove; //0x00A4
	float timeThrottlingAfterDownShift; //0x00F4
	float amountThrottlingAfterDownShift; //0x00F8
	float minSpeedForThrottlingAfterDownShift; //0x00FC
}; //Size: 0x0100

class EngineStaticState
{
public:
	float powerToFrontWheels; //0x0000
	float powerToRearWheels; //0x0004
	float idle; //0x0008
	float redLine; //0x000C
	float maxRPM; //0x0010
	float flyWheelMass; //0x0014
	float frontWheelRadius; //0x0018
	float rearWheelRadius; //0x001C
	class PointGraph8 torqueCurve; //0x0020
	class PointGraph8 torqueFrictionCurve; //0x0070
	float unloadedEngineFriction; //0x00C0
	float revLimiterTime; //0x00C4
	class PointGraph8 engineBrakingVsGear; //0x00C8
	float gearRatioValues[10]; //0x0118
	uint32_t numberOfGearRatiosValues; //0x0140
	float gearEfficiencyValues[10]; //0x0144
	uint32_t numberOfGearEfficiencyValues; //0x016C
	float peakTorqueRPM; //0x0170
	float peakTorque; //0x0174
	float clutchSlip; //0x0178
	float finalGear; //0x017C
	float antiBogMaxMultiplier; //0x0180
	float antiBogRampTime; //0x0184
	float speedLimiter; //0x0188
	float speedLimiterNos; //0x018C
	float reverseSpeedLimiter; //0x0190
	float reverseTorqueMultiplier; //0x0194
	class PointGraph8 tractionControlScalarVsSpeed; //0x0198
	class PointGraph8 tractionControlScalarVsSlipAngle; //0x01E8
	float engineLoadLerp; //0x0238
	float minLoadAtTopSpeed; //0x023C
	float fullLoadTorqueNoise; //0x0240
	float zeroLoadTorqueNoise; //0x0244
	class PointGraph8 torqueNoise; //0x0248
}; //Size: 0x0298

class ForcedInductionStaticState : public CommonVehicleState
{
public:
	float minRPM; //0x0004
	float maxRPM; //0x0008
	float highBoost; //0x000C
	float lowBoost; //0x0010
	eInductionType inductionType; //0x0014
	float boostStartRPM; //0x0018
	float boostAlwaysThrottle; //0x001C
	float boostMinThrottle; //0x0020
	float peakPSI; //0x0024
	bool hasBypassValve; //0x0028
	bool hasBlowoffValve; //0x0029
	char pad_002A[2]; //0x002A
	float lagTime; //0x002C
	float spinDownTime; //0x0030
}; //Size: 0x0034

class SequenceManager
{
public:
	bool mPlayingSequences[2]; //0x0000
	char pad_0002[2]; //0x0002
}; //Size: 0x0004

class Sequencer
{
public:
	float mCue; //0x0000
	float mSpeed; //0x0004
	float mTotalLength; //0x0008
	bool mReachedEnd; //0x000C
	char pad_000D[3]; //0x000D
}; //Size: 0x0010

class PowerTrainInternalState
{
public:
	float mClutchGrinding; //0x0000
	int32_t mClutchGrindImpulse; //0x0004
	float mMaxHP; //0x0008
	float mMaxHorsePowerRPM; //0x000C
	float mPeakTorque; //0x0010
	float mPeakTorqueRPM; //0x0014
	bool mBlown; //0x0018
	char pad_0019[3]; //0x0019
	float mThrottle; //0x001C
	float mClutchPedal; //0x0020
	float mDeltaTime; //0x0024
}; //Size: 0x0028

class CorePowerTrain : public ShiftingAIStaticState
{
public:
	class ThrottleAIStaticState mThrottleAIStaticState; //0x0080
	class EngineStaticState mEngineStaticState; //0x0180
	class ForcedInductionStaticState mForcedInductionStaticState; //0x0418
	class SequenceManager mPlayingSequences; //0x044C
	class Sequencer mUpShiftSequence; //0x0450
	class Sequencer mDownShiftSequence; //0x0460
	class PowerTrainInternalState mInternal; //0x0470
}; //Size: 0x0498

class ChassisOutputState
{
public:
	__m128 forceAppliedToCar; //0x0000
	__m128 torqueAppliedToCar; //0x0010
	__m128 localCOG; //0x0020
	float local_steeringWheelAngles[2]; //0x0030
	char pad_0038[8]; //0x0038
	__m128 tireLateralNormal[4]; //0x0040
	__m128 tireSteeringNormal[4]; //0x0080
	float suspensionCompression[4]; //0x00C0
	__m128 world_wheelTirePatchPosition[4]; //0x00D0
	__m128 local_wheelTirePatchPosition[4]; //0x0110
	bool suspensionIsAtRest; //0x0150
	bool vehicleIsAsleep; //0x0151
	char pad_0152[14]; //0x0152
	__m128 aeroSideForce; //0x0160
	__m128 aeroDownForce; //0x0170
	__m128 aeroDragForce; //0x0180
	__m128 aeroDownCenter; //0x0190
	__m128 aeroSideCenter; //0x01A0
	__m128 aeroDragCenter; //0x01B0
	__m128 aeroDragTorque; //0x01C0
	float ffbTorque; //0x01D0
	float aeroDraftGauge; //0x01D4
	float aeroDraftSlingshot; //0x01D8
	float aeroDraftHasFilledGauge; //0x01DC
	bool isABSOn[4]; //0x01E0
	char pad_01E4[4]; //0x01E4
}; //Size: 0x01E8

class ChassisResult : public ChassisOutputState
{
public:
	char pad_01E8[8]; //0x01E8
	__m128 tireGroundNormal[4]; //0x01F0
	__m128 driveshaftTorqueOnCarBody; //0x0230
	float tireLateralForce[4]; //0x0240
	float tireLongitudinalForce[4]; //0x0250
	__m128 tireForces[4]; //0x0260
	__m128 tireTorques[4]; //0x02A0
	float tireSlipAngle[4]; //0x02E0
	float tireCamber[4]; //0x02F0
	float tireLongSlip[4]; //0x0300
	float tireTraction[4]; //0x0310
	float tireTorque[4]; //0x0320
	float tireIdealTorque[4]; //0x0330
	float tireBrakeTorque[4]; //0x0340
	float tireAngularVelocity[4]; //0x0350
	float tireForwardVelocity[4]; //0x0360
	float tireLateralVelocity[4]; //0x0370
	float tireSlipRatio[4]; //0x0380
	float tireRadius[4]; //0x0390
	bool tireIsOnMovingGround[4]; //0x03A0
	bool isWheelOnGround[4]; //0x03A4
	int32_t numberOfWheelsOnGround; //0x03A8
	float suspensionSpringForce[4]; //0x03AC
	float suspensionLoad[4]; //0x03BC
	float suspensionDigression[4]; //0x03CC
	float suspensionMaxDelta; //0x03DC
	float driveShaftTorque; //0x03E0
	float wheelieAngle; //0x03E4
	bool staticResetConditionMet; //0x03E8
	char pad_03E9[7]; //0x03E9
}; //Size: 0x03F0

class SteeringResultState
{
public:
	float N00000B88; //0x0000
	float steeringSpeed; //0x0004
	float steeringRangeMultiplier; //0x0008
	float steeringRangeLeft; //0x000C
	float steeringRangeRight; //0x0010
	float N00001105; //0x0014
	float N00001FE6; //0x0018
}; //Size: 0x001C

class RaceCarExtraForcesResult
{
public:
	__m128 extraTorque; //0x0000
	__m128 extraForce; //0x0010
	float extraFishTailDirection; //0x0020
	float extraFishTailTimer; //0x0024
	char pad_0028[8]; //0x0028
}; //Size: 0x0030

class VehiclePhysicsSomething
{
public:
	char pad_0000[44]; //0x0000
}; //Size: 0x002C

class PowerTrainResultState
{
public:
	GearID gear; //0x0000
	float torqueSplit; //0x0004
	char pad_0008[12]; //0x0008
	float engineTorque; //0x0014
	float maxHorsePower; //0x0018
	float maxHorsePowerRPM; //0x001C
	float minHorsePower; //0x0020
	float currentHorsePower; //0x0024
	float engineRpm; //0x0028
	float driveShaftRpm; //0x002C
	float peakTorque; //0x0030
	float peakTorqueRPM; //0x0034
	float throttle; //0x0038
	float clutch; //0x003C
	ClutchState clutchState; //0x0040
	float torqueRatio; //0x0044
	float clutchGrind; //0x0048
	float forcedIndSpool; //0x004C
	float forcedIndCurrentPSI; //0x0050
	float forcedIndRelativeTorqueGain; //0x0054
	float forcedIndByPassPosition; //0x0058
	bool forcedIndIsBlowoffOpened; //0x005C
	bool forcedIndIsWestgateOpened; //0x005D
	bool isGearChanging; //0x005E
	char pad_005F[1]; //0x005F
}; //Size: 0x0060

class StabilityManagementStaticState : public PointGraph8
{
public:
	class PointGraph8 ESCOverSteerDecMinAcc; //0x0050
	class PointGraph8 ESCUnderSteerMinAcc; //0x00A0
	class PointGraph8 ESCOverSteerFrontBrake; //0x00F0
	class PointGraph8 ESCOverSteerRearBrake; //0x0140
	class PointGraph8 ESCUnderSteerFrontBrake; //0x0190
	class PointGraph8 ESCUnderSteerRearBrake; //0x01E0
	class PointGraph8 ESCUnderSteerTorque; //0x0230
}; //Size: 0x0280

class ChassisStaticState
{
public:
	__m128 dimension; //0x0000
	__m128 local_wheelMountPosition[4]; //0x0010
	class StabilityManagementStaticState stabilityManagementStaticState; //0x0050
	float ride_height_specs[2]; //0x02D0
	float travel_specs[2]; //0x02D8
	float spring_specs[2]; //0x02E0
	float springProgression[2]; //0x02E8
	class AxlePairPointGraph8 sway_specs; //0x02F0
	class AxlePairPointGraph8 shock_specs; //0x0390
	class AxlePairPointGraph8 shock_ext_specs; //0x0430
	float shock_valving[2]; //0x04D0
	float shock_digression[2]; //0x04D8
	float shockBlowout; //0x04E0
	class AxlePairPointGraph8 brakeSpec; //0x04E4
	class PointGraph8 eBrakeSpec; //0x0584
	float brakeScalarWhenSteering[2]; //0x05D4
	float brakeScalarDeadZone; //0x05DC
	float differentialFactor[3]; //0x05E0
	float dragCoefficient; //0x05EC
	float nitrousDragCoefficient; //0x05F0
	float aeroTopSpeed; //0x05F4
	float aeroNitrousTopSpeed; //0x05F8
	class PointGraph8 aeroCG; //0x05FC
	class PointGraph8 aeroCoefficient; //0x064C
	float groundEffectHeightRatio; //0x069C
	float liftHeightRatio; //0x06A0
	float aeroCoefficientMaxGroundEffect; //0x06A4
	float aeroCoefficientMaxLift; //0x06A8
	float aeroCoefficientOffThrottle; //0x06AC
	float flyingCarAeroCoefficient_BigAir; //0x06B0
	float flyingCarAeroCoefficient_SmallAir; //0x06B4
	class PointGraph8 steeringTuning; //0x06B8
	float wheelBase; //0x0708
	float trackWidth[2]; //0x070C
	float sectionWidth[2]; //0x0714
	float frontAxle; //0x071C
	class PointGraph8 rollCenter; //0x0720
	class AxlePairPointGraph8 toe; //0x0770
	class AxlePairPointGraph8 camber; //0x0810
	class PointGraph8 caster; //0x08B0
	float NOSGripIncrease; //0x0900
	float staticGripOffThrottle; //0x0904
	float driveshaftTorqueEffect; //0x0908
	float driveshaftTorqueMax; //0x090C
	class PointGraph8 frontWeightBias; //0x0910
	class PointGraph8 gripVsBrake; //0x0960
	float gripVsBrakeBlend; //0x09B0
	float ABSTorqueRatioLowerBound; //0x09B4
	float ABSTorqueRatioUpperBound; //0x09B8
	float ABSSlipRatioSystemTargetSR; //0x09BC
	bool ABSSlipRatioSystemEnabled; //0x09C0
	char pad_09C1[3]; //0x09C1
	class PointGraph8 ABSTorqueRatioScalarVsSpeed; //0x09C4
	class PointGraph8 steerFFBVsSlipAngle; //0x0A14
	class PointGraph8 steerFFBVsSpeed; //0x0A64
	float downForce; //0x0AB4
	float downForceOffset; //0x0AB8
	float downForceOffsetUnderBraking; //0x0ABC
	float downForceOffsetInDrift; //0x0AC0
	int32_t tireModelType; //0x0AC4
	char pad_0AC8[8]; //0x0AC8
}; //Size: 0x0AD0

class PowerTrainStaticState
{
public:
	float powerToFrontWheels; //0x0000
	float powerToRearWheels; //0x0004
	float idle; //0x0008
	float redLine; //0x000C
	float maxRPM; //0x0010
	float flyWheelMass; //0x0014
	float frontWheelRadius; //0x0018
	float rearWheelRadius; //0x001C
	class PointGraph8 torqueCurve; //0x0020
	class PointGraph8 torqueFrictionCurve; //0x0070
	class PointGraph8 engineBrakingVsGear; //0x00C0
	float unloadedEngineFriction; //0x0110
	float revLimiterTime; //0x0114
	float gearRatioValues[10]; //0x0118
	uint32_t numberOfGearRatioValues; //0x0140
	float gearEfficiencyValues[10]; //0x0144
	uint32_t numberOfGearEfficiencyValues; //0x016C
	float peakTorqueRPM; //0x0170
	float clutchSlip; //0x0174
	float finalGear; //0x0178
	float antiBogMaxMultiplier; //0x017C
	float antiBogRampTime; //0x0180
	float speedLimiter; //0x0184
	float speedLimiterNos; //0x0188
	float reverseSpeedLimiter; //0x018C
	float reverseTorqueMultiplier; //0x0190
	class PointGraph8 tractionControlScalarVsSpeed; //0x0194
	class PointGraph8 tractionControlScalarVsSlipAngle; //0x01E4
	GearID topGear; //0x0234
	uint32_t numberOfShiftUpRPMValues; //0x0238
	float shiftUpRPMValues[10]; //0x023C
	uint32_t numberOfShiftDownRPMValues; //0x0264
	float shiftDownRPMValues[10]; //0x0268
	float minTireTractionToShiftUpFirstGear; //0x0290
	float minTireTractionToShiftUp; //0x0294
	float shiftDownBrakeDynamicFactor; //0x0298
	class PointGraph8 UpShiftThrottle; //0x029C
	class PointGraph8 UpShiftClutch; //0x02EC
	class PointGraph8 DownShiftThrottle; //0x033C
	class PointGraph8 DownShiftClutch; //0x038C
	class PointGraph8 DownShiftSlopeScalar; //0x03DC
	float forcedIndHighBoost; //0x042C
	float forcedIndLowBoost; //0x0430
	eInductionType forcedInductionType; //0x0434
	float forcedIndBoostStartRPM; //0x0438
	float forcedIndBoostAlwaysThrottle; //0x043C
	float forcedIndBoostMinThrottle; //0x0440
	float forcedIndPeakPSI; //0x0444
	bool forcedIndHasBypassValve; //0x0448
	bool forcedIndHasBlowoffValve; //0x0449
	char pad_044A[2]; //0x044A
	float forcedIndLagTime; //0x044C
	float forcedIndSpinDownTime; //0x0450
	class PointGraph8 throttleMappingFirstGear; //0x0454
	class PointGraph8 throttleMappingSecondGear; //0x04A4
	class PointGraph8 throttleMappingThirdAndAbove; //0x04F4
	float timeThrottlingAfterDownShift; //0x0544
	float amountThrottlingAfterDownShift; //0x0548
	float minSpeedForThrottlingAfterDownShift; //0x054C
	float engineLoadLerp; //0x0550
	float minLoadAtTopSpeed; //0x0554
	float zeroLoadTorqueNoise; //0x0558
	float fullLoadTorqueNoise; //0x055C
	class PointGraph8 torqueNoise; //0x0560
}; //Size: 0x05B0

class SteeringOutputState
{
public:
	float mSteering; //0x0000
	float mSteeringSpeed; //0x0004
}; //Size: 0x0008

class SteeringFeedbackState : public SteeringOutputState
{
public:
	float mSteeringInput; //0x0008
	float mSteeringRangeLeft; //0x000C
	float mSteeringRangeRight; //0x0010
}; //Size: 0x0014

class DrivingHistory
{
public:
	float mSmoothedGas; //0x0000
	float mSmoothedBrake; //0x0004
	float mSmoothedSteering; //0x0008
	float mTimeSinceDownShifted; //0x000C
	float mTimeSinceUpShifted; //0x0010
}; //Size: 0x0014

class PowerTrainFeedbackState
{
public:
	GearID gear; //0x0000
	ClutchState clutchState; //0x0004
	float clutch; //0x0008
	float noShiftAllowedTimer; //0x000C
	float throttle; //0x0010
	float spool; //0x0014
	float psi; //0x0018
	float rpm; //0x001C
	float rpmAntiBogging; //0x0020
	float engineAngularVelocity; //0x0024
	float driveShaftAngularVelocity; //0x0028
	float revLimiterTimeLeft; //0x002C
	float engineLoad; //0x0030
	float engineTime; //0x0034
	class DrivingHistory mDrivingHistory; //0x0038
	class PowerTrainInternalState mInternalState; //0x004C
}; //Size: 0x0074

class TireFeedbackState
{
public:
	float angularVelocity; //0x0000
	float slipRatio; //0x0004
	float idealTorque; //0x0008
	float slip; //0x000C
	float forwardVelocity; //0x0010
	float lateralVelocity; //0x0014
}; //Size: 0x0018

class AerodynamicsFeedback
{
public:
	float draftingTime; //0x0000
	float bigAirTimer; //0x0004
	float inAirTimer; //0x0008
	bool isDoingBigAir; //0x000C
	bool hasFilledDraftingGauge; //0x000D
	char pad_000E[2]; //0x000E
}; //Size: 0x0010

class ChassisFeedbackState : public ChassisInternalState
{
public:
	__m128 world_wheelTirePatchPosition[4]; //0x0060
	int32_t numberOfWheelsOnGround; //0x00A0
	float compression[4]; //0x00A4
	float antiSwayVelocity[2]; //0x00B4
	class TireFeedbackState wheelFeedback[4]; //0x00BC
	class AerodynamicsFeedback aerodynamicsFeedback; //0x011C
	float tireLateralForce[4]; //0x012C
	float tireSlipAngle[4]; //0x013C
	bool N00001A72; //0x014C
	char pad_014D[3]; //0x014D
	__m128 tireForces[4]; //0x0150
	__m128 tireTorques[4]; //0x0190
	bool isWheelOnGround[4]; //0x01D0
	float tireLongSlip[4]; //0x01D4
	float tireTraction[4]; //0x01E4
	float tireTorque[4]; //0x01F4
	float tireIdealTorque[4]; //0x0204
	float tireBrakeTorque[4]; //0x0214
	float tireAngularVelocity[4]; //0x0224
	float tireLastSlipAngle[4]; //0x0234
	float escUnderSteerTimer; //0x0244
}; //Size: 0x0248

class RaceCar : public OutputState
{
public:
	class TuningState mVehicleTuning; //0x0380
	class SteeringStaticState mSteeringStaticState; //0x2060
	char pad_23A4[12]; //0x23A4
	class CoreChassis mChassis; //0x23B0
	class CorePowerTrain mPowerTrain; //0x34C0
	char pad_3958[8]; //0x3958
	class ChassisResult mChassisResult; //0x3960
	class SteeringResultState mSteeringResultState; //0x3D50
	char pad_3D6C[4]; //0x3D6C
	class RaceCarExtraForcesResult mRaceCarExtraForcesResult; //0x3D70
	class VehiclePhysicsSomething something; //0x3DA0
	class PowerTrainResultState mPowerTrainResult; //0x3DCC
	char pad_3E2C[36]; //0x3E2C
	class ChassisStaticState mChassisStaticState; //0x3E50
	class PowerTrainStaticState mPowerTrainStaticState; //0x4920
	class SteeringFeedbackState mSteeringFeedback; //0x4ED0
	class PowerTrainFeedbackState mPowerTrainFeedback; //0x4EE4
	char pad_4F58[8]; //0x4F58
	class ChassisFeedbackState mChassisFeedback; //0x4F60
	char pad_51A8[32]; //0x51A8
}; //Size: 0x51C8