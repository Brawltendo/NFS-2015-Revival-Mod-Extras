#pragma once

enum TireModelType : __int32
{
	TireModelType_Analytical = 0x0,
	TireModelType_Pacejka52 = 0x1,
	TireModelType_DoT = 0x2,
};

enum VehicleMode : __int32
{
	VmIdle = 0x0,
	VmEntering = 0x1,
	VmEntered = 0x2,
	VmStarting = 0x3,
	VmStarted = 0x4,
	VmStopping = 0x5,
	VmLeaving = 0x6,
};


enum GearID : __int32
{
	G_REVERSE = 0x0,
	G_NEUTRAL = 0x1,
	G_FIRST = 0x2,
	G_SECOND = 0x3,
	G_THIRD = 0x4,
	G_FOURTH = 0x5,
	G_FIFTH = 0x6,
	G_SIXTH = 0x7,
	G_SEVENTH = 0x8,
	G_EIGHTH = 0x9,
	G_MAX = 0xA,
};

enum ForcedInductionType : __int32
{
	ForcedInductionType_None = 0x0,
	ForcedInductionType_TurboCharger = 0x1,
	ForcedInductionType_RootsBlower = 0x2,
	ForcedInductionType_CentrifugalBlower = 0x3,
	ForcedInductionType_Twinscrew = 0x4,
};


enum eInductionType : __int32
{
	INDUCTION_NONE = 0x0,
	INDUCTION_TURBO_CHARGER = 0x1,
	INDUCTION_ROOTS_BLOWER = 0x2,
	INDUCTION_CENTRIFUGAL_BLOWER = 0x3,
	INDUCTION_TWINSCREW_BLOWER = 0x4,
};

enum SteeringType : __int32
{
	kGamePad = 0x0,
	kWheelMultiTurn = 0x1,
	kWheelNotMultiTurn = 0x2,
};

enum SleepState : __int32
{
	SS_NONE = 0x0,
	SS_ALL = 0x1,
	SS_LATERAL = 0x2,
};

enum ClutchState : __int32
{
	CLUTCH_STATE_ENGAGED = 0x0,
	CLUTCH_STATE_DISENGAGED = 0x1,
	CLUTCH_STATE_UPSHIFT = 0x2,
	CLUTCH_STATE_DOWNSHIFT = 0x3,
};

enum ShiftStatus : __int32
{
	SHIFT_STATUS_NONE = 0x0,
	SHIFT_STATUS_NORMAL = 0x1,
	SHIFT_STATUS_GOOD = 0x2,
	SHIFT_STATUS_PERFECT = 0x3,
	SHIFT_STATUS_MISSED = 0x4,
};

enum ShiftPotential : __int32
{
	SHIFT_POTENTIAL_NONE = 0x0,
	SHIFT_POTENTIAL_DOWN = 0x1,
	SHIFT_POTENTIAL_UP = 0x2,
	SHIFT_POTENTIAL_GOOD = 0x3,
	SHIFT_POTENTIAL_PERFECT = 0x4,
	SHIFT_POTENTIAL_MISS = 0x5,
};

enum EDriverComponentState : __int32
{
	eDriverComponentStateEngineOff = 0x0,
	eDriverComponentStateIgnition = 0x1,
	eDriverComponentStateIdle = 0x2,
	eDriverComponentStateLeavingIdle = 0x3,
	eDriverComponentStateLeavingIdleCarRolling = 0x4,
	eDriverComponentStateMoving = 0x5,
	eDriverComponentStateChangingGearDepressingClutch = 0x6,
	eDriverComponentStateChangingGearHoldingClutch = 0x7,
	eDriverComponentStateChangingGearLiftingClutch = 0x8,
	eDriverComponentStateHandbrake = 0x9,
	eDriverComponentStateChangingDownGearDepressingClutch = 0xA,
	eDriverComponentStateChangingDownGearHoldingClutch = 0xB,
	eDriverComponentStateChangingDownGearLiftingClutch = 0xC,
	eDriverComponentStateSlowingDown = 0xD,
	eDriverComponentStateMax = 0xE,
};

enum NFSVehicleState : __int32
{
	NFSVehicleState_OnGround = 0x0,
	NFSVehicleState_InAir = 0x1,
	NFSVehicleState_Landing = 0x2,
	NFSVehicleState_Tumbling = 0x3,
	NFSVehicleState_Collided = 0x4,
	NFSVehicleState_Totalled = 0x5,
	NFSVehicleState_StartTumble = 0x6,
	NFSVehicleState_Dead = 0x7,
};

enum EHandbrakeState : __int32
{
	EHandbrakeStateOff = 0x0,
	EHandbrakeStateOnForwards = 0x1,
	EHandbrakeStateTurning = 0x2,
	EHandbrakeStateTurnComplete = 0x3,
	EHandbrakeStateOnBackwards = 0x4,
	EHandbrakeStateJTurnComplete = 0x5,
};

enum RaceVehicleModificationType : __int32
{
	ModificationType_Scalar = 0x0,
	ModificationType_Addition = 0x1,
	ModificationType_Override = 0x2,
};

enum RaceVehicleAttributeToModify : __int32
{
	ATM_EngineTorque = 0x0,
	ATM_DriftYawTorque = 0x1,
	ATM_FrontDrag = 0x2,
	ATM_XCarMultiplier = 0x3,
	ATM_SuspensionStiffness_Front = 0x4,
	ATM_SuspensionStiffness_Rear = 0x5,
	ATM_TorqueSplit = 0x6,
	ATM_DiameterFront = 0x7,
	ATM_DiameterRear = 0x8,
	ATM_RimDiameterFront = 0x9,
	ATM_RimDiameterRear = 0xA,
	ATM_RevLimiterTime = 0xB,
	ATM_Mass = 0xC,
	ATM_Strength = 0xD,
	ATM_GearChangeTime = 0xE,
	ATM_DownForceEffectOnBody = 0xF,
	ATM_FrontFinalLongGripForZeroSlip = 0x10,
	ATM_FrontSlipInfluenceOnLongGrip = 0x11,
	ATM_RearFinalLongGripForZeroSlip = 0x12,
	ATM_RearSlipInfluenceOnLongGrip = 0x13,
	ATM_RollAngleLimit = 0x14,
	ATM_MaintainSpeedInDriftAmount = 0x15,
	ATM_MinTireTractionToShiftUp = 0x16,
	ATM_MinTireTractionToShiftUpFirstGear = 0x17,
	ATM_GravityMultiplier = 0x18,
	ATM_Downforce = 0x19,
	ATM_MinimumAngleForDrift = 0x1A,
	ATM_SideForceMagnitude = 0x1B,
	ATM_FrontDragSpeedThreshold = 0x1C,
	ATM_NitrousFrontDragSpeedThreshold = 0x1D,
	ATM_FinalDriveRatio = 0x1E,
	ATM_GearDownRPM = 0x1F,
	ATM_GearUpSpeed = 0x20,
	ATM_GearEfficiency0 = 0x21,
	ATM_GearEfficiency1 = 0x22,
	ATM_GearEfficiency2 = 0x23,
	ATM_GearEfficiency3 = 0x24,
	ATM_GearEfficiency4 = 0x25,
	ATM_GearEfficiency5 = 0x26,
	ATM_GearEfficiency6 = 0x27,
	ATM_GearEfficiency7 = 0x28,
	ATM_GearEfficiency8 = 0x29,
	ATM_GearEfficiency9 = 0x2A,
	ATM_GearEfficiency10 = 0x2B,
	ATM_LongitudinalFrictionScaleFront = 0x2C,
	ATM_LongitudinalFrictionScaleRear = 0x2D,
	ATM_SpeedLimiter = 0x2E,
	ATM_SpeedLimiterNOS = 0x2F,
	ATM_BurnoutTendency = 0x30,
	ATM_GentleInputSteerRate = 0x31,
	ATM_VelocityDragMultiplier = 0x32,
	ATM_SlipAngleForMinDragMultiplier = 0x33,
	ATM_SlipAngleForMaxDragMultiplier = 0x34,
	ATM_MinDragMultiplier = 0x35,
	ATM_MaxDragMultiplier = 0x36,
	ATM_TopGear = 0x37,
	ATM_LateralFrictionScaleFront = 0x38,
	ATM_LateralFrictionScaleRear = 0x39,
	ATM_FrontBrakeTorqueVsSpeed = 0x3A,
	ATM_RearBrakeTorqueVsSpeed = 0x3B,
	ATM_SteerRate = 0x3C,
	ATM_XCarWheelCamberBottomFront = 0x3D,
	ATM_XCarWheelCamberBottomRear = 0x3E,
	ATM_XCarWheelCamberTopFront = 0x3F,
	ATM_XCarWheelCamberTopRear = 0x40,
	ATM_XCarWheelCasterFront = 0x41,
	ATM_GearRatio0 = 0x42,
	ATM_GearRatio1 = 0x43,
	ATM_GearRatio2 = 0x44,
	ATM_GearRatio3 = 0x45,
	ATM_GearRatio4 = 0x46,
	ATM_GearRatio5 = 0x47,
	ATM_GearRatio6 = 0x48,
	ATM_GearRatio7 = 0x49,
	ATM_GearRatio8 = 0x4A,
	ATM_GearRatio9 = 0x4B,
	ATM_GearRatio10 = 0x4C,
	ATM_DownforceOffsetUnderBraking = 0x4D,
	ATM_HandbrakeTorqueVsSpeed = 0x4E,
	ATM_CounterSteerRate = 0x4F,
	ATM_CenterSteerRate = 0x50,
	ATM_SteeringRangeOffThrottle = 0x51,
	ATM_SteeringRangeOnThrottle = 0x52,
	ATM_ReverseSteeringRange = 0x53,
	ATM_DriftCounterSteerRate = 0x54,
	ATM_MaxSteeringAngle = 0x55,
	ATM_MinHandbrakeTime = 0x56,
	ATM_TrackWidthFront = 0x57,
	ATM_TrackWidthRear = 0x58,
	ATM_CamberFront = 0x59,
	ATM_CamberRear = 0x5A,
	ATM_RideHeightSpecsFront = 0x5B,
	ATM_RideHeightSpecsRear = 0x5C,
	ATM_ToeFront = 0x5D,
	ATM_ToeRear = 0x5E,
	ATM_ReduceForwardSpeedAmount = 0x5F,
	ATM_SlipAngleToStartBlendingDownDriftScale = 0x60,
	ATM_SlipAngleForZeroDriftScale = 0x61,
	ATM_DriftScaleFromBraking = 0x62,
	ATM_SlipAngleToEnterWhenBraking = 0x63,
	ATM_SlipAngleToEnterWhenHandbraking = 0x64,
	ATM_SlipAngleToEnterWhenBrakeStab = 0x65,
	ATM_SlipAngleToEnterWhenScandinavianFlick = 0x66,
	ATM_StartingDriftScaleAfterScandinavianFlick = 0x67,
	ATM_DriftAngleToExitDrift = 0x68,
	ATM_DriftAngularDamping = 0x69,
	ATM_DriftScaleFromHandbrake = 0x6A,
	ATM_DriftScaleDecay = 0x6B,
	ATM_DriftScaleFromSteering = 0x6C,
	ATM_DriftScaleFromCounterSteering = 0x6D,
	ATM_DriftScaleFromGasLetOff = 0x6E,
	ATM_DriftSidewaysDamping = 0x6F,
	ATM_DonutLongitudinalGripMin = 0x70,
	ATM_DonutLongitudinalGripMax = 0x71,
	ATM_DonutLateralGripMin = 0x72,
	ATM_DonutLateralGripMax = 0x73,
	ATM_DifferentialFront = 0x74,
	ATM_DifferentialCenter = 0x75,
	ATM_DifferentialRear = 0x76,
	ATM_NOS_DragCoefficient = 0x77,
	ATM_XCarYOffset = 0x78,
	ATM_XCarBodyYOffset = 0x79,
	ATM_XCarRideHeightFront = 0x7A,
	ATM_XCarRideHeightRear = 0x7B,
	ATM_XCarWheelXOffsetBottomFront = 0x7C,
	ATM_XCarWheelXOffsetTopFront = 0x7D,
	ATM_XCarWheelXOffsetBottomRear = 0x7E,
	ATM_XCarWheelXOffsetTopRear = 0x7F,
	ATM_XCarToeAngleFront = 0x80,
	ATM_XCarToeAngleRear = 0x81,
	ATM_DefaultSteering = 0x82,
	ATM_DriftScaleFromHandbrakeAtLowSpeed = 0x83,
	ATM_ExtraFishTailTorque = 0x84,
	ATM_GasLetOffYawTorque = 0x85,
	ATM_DefaultSteeringRemapping = 0x86,
	ATM_CounterSteeringRemapping = 0x87,
	ATM_XCarScalarX = 0x88,
	ATM_XCarScalarY = 0x89,
	ATM_XCarScalarZ = 0x8A,
	ATM_DownforceOffset = 0x8B,
	ATM_DownforceOffsetInDrift = 0x8C,
	ATM_BackwardsExtraBrakeStrength = 0x8D,
	ATM_LongGripForZeroSlipAnglePeakGripFront = 0x8E,
	ATM_BrakingLongGripForZeroSlipAnglePeakGripFront = 0x8F,
	ATM_BrakingLongGripForZeroSlipAngleFinalGripFront = 0x90,
	ATM_LongGripForZeroSlipAnglePeakGripRear = 0x91,
	ATM_BrakingLongGripForZeroSlipAnglePeakGripRear = 0x92,
	ATM_BrakingLongGripForZeroSlipAngleFinalGripRear = 0x93,
	ATM_TorqueSplitInDrift = 0x94,
	ATM_BrakingLateralGripForLargeSlipRatioInitialGripFront = 0x95,
	ATM_BrakingLateralGripForLargeSlipRatioInitialGripRear = 0x96,
	ATM_BrakingLateralGripForLargeSlipRatioPeakGripFront = 0x97,
	ATM_BrakingLateralGripForLargeSlipRatioPeakGripRear = 0x98,
	ATM_AligningTorqueEffectInDrift = 0x99,
	ATM_SlipAngleForFullDampingFade = 0x9A,
	ATM_DriftScaleFromGasStab = 0x9B,
	ATM_WheelPivotPointXOffsetFront = 0x9C,
	ATM_WheelPivotPointXOffsetRear = 0x9D,
	ATM_Max = 0x9E,
};

enum VehicleCollisionBody : __int32
{
	VehicleCollisionBody_StockOnGround = 0x0,
	VehicleCollisionBody_StockCrashing = 0x1,
	VehicleCollisionBody_WideOnGround = 0x2,
	VehicleCollisionBody_WideCrashing = 0x3,
};
