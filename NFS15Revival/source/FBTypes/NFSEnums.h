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
