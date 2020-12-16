#pragma once
#include "ConfigData.h"

class DonutComponent
{
public:
	class DriftParams* driftParams; //0x0000
	class RaceRigidBody* rigidBody; //0x0008
	__m128 donutScale; //0x0010
	__m128 timeSteeringWhileOnThrottle; //0x0020
	__m128 donutGrip; //0x0030
	char pad_0040[88]; //0x0040
}; //Size: 0x0098

class DriftComponent
{
public:
	char pad_0000[20]; //0x0000
	bool isDrifting; //0x0014
	bool counterSteeringInDrift; //0x0015
	char pad_0016[2]; //0x0016
	class RaceRigidBody* mpChassisRigidBody; //0x0018
	class DriftParams* mpParams; //0x0020
	char pad_0028[8]; //0x0028
	float currentYawTorque; //0x0030
	__int32 someEnum; //0x0034
	char pad_0038[24]; //0x0038
	__m128 mvfTimeSinceExittingDrift; //0x0050
	__m128 mvfMaintainedSpeed; //0x0060
	__m128 mvfDriftYawDamping; //0x0070
	__m128 mvfDriftScale; //0x0080
	__m128 mvfTimeDrifting; //0x0090
	__m128 mvfSideForceMagnitude; //0x00A0
	__m128 mvfPropSpeedMaintainAlongZ; //0x00B0
	__m128 mvfPropSpeedMaintainAlongVel; //0x00C0
	__m128 mvfLatDriftForceFactor; //0x00D0
	__m128 mvfCurrentDriftAngle; //0x00E0
	__m128 mvfTimeInDriftWithStaticFriction; //0x00F0
	__m128 mvfCounterSteerSideMag; //0x0100
	__m128 mvfRearSlipAngle; //0x0110
	__m128 mv_YawDamping_Spare_Spare_Spare; //0x0120
	__m128 mv_MaintainSpeedForce_SideForce_Spare_Spare; //0x0130
	__m128 mvfPreviousGasInput; //0x0140
	__m128 mvfMaxSlipAngle; //0x0150
	__m128 mvfSlipAngleForDriftEntry; //0x0160
	__m128 mvfTimePressingBrake; //0x0170
	__m128 mvbPressedFullBrake; //0x0180
	__m128 mvfTimeWithLowSlipAngle; //0x0190
	__m128 mvbDriftingWithLowSlipAngle; //0x01A0
	__m128 mvfTimeSteerLeft; //0x01B0
	__m128 mvfLastTimeSteerLeft; //0x01C0
	__m128 mvfTimeSinceSteerLeft; //0x01D0
	__m128 mvfTimeSteerRight; //0x01E0
	__m128 mvfLastTimeSteerRight; //0x01F0
	__m128 mvfTimeSinceSteerRight; //0x0200
	__m128 mvfTireGrip; //0x0210
	class PerformanceModificationComponent* m_performanceModificationComponent; //0x0220
	char pad_0228[52]; //0x0228
}; //Size: 0x025C

class DriftParams
{
public:
	class RaceVehicleDriftConfigData* driftTriggerParams; //0x0000
	class PerformanceModificationComponent* N00000F69; //0x0008
	class RaceVehicleDriftConfigData* driftScaleParams; //0x0010
	class PerformanceModificationComponent* pad_0018; //0x0018
	class RaceVehicleDriftConfigData* yawTorqueParams; //0x0020
	class PerformanceModificationComponent* pad_0028; //0x0028
	class RaceVehicleDriftConfigData* sideForceParams; //0x0030
	class PerformanceModificationComponent* pad_0038; //0x0038
	class RaceVehicleDriftConfigData* otherParams; //0x0040
	class PerformanceModificationComponent* pad_0048; //0x0048
	class RaceVehicleDriftConfigData* exitParams; //0x0050
	class PerformanceModificationComponent* pad_0058; //0x0058
	class RaceVehicleDriftConfigData* donutParams; //0x0060
	char pad_0068[32]; //0x0068
}; //Size: 0x0088

class DriverComponent
{
public:
	char pad_0000[16]; //0x0000
	float gasInput; //0x0010
	float brakeInput; //0x0014
	char pad_0018[40]; //0x0018
	__m128 N000107FD; //0x0040
	__m128 N000107FF; //0x0050
	__m128 N00010801; //0x0060
	__m128 N00010803; //0x0070
	char pad_0080[8]; //0x0080
	float timeSinceClutchEngaged; //0x0088
	char pad_008C[8]; //0x008C
	float timeSinceGearChange; //0x0094
	char pad_0098[120]; //0x0098
	__m128 shiftUpRPM; //0x0110
	char pad_0120[32]; //0x0120
	GearID gear; //0x0140
	char pad_0144[4]; //0x0144
	class TuningState* tuningState; //0x0148
	EDriverComponentState driverState; //0x0150
	char pad_0154[8]; //0x0154
	bool N0001087D; //0x015C
	bool N00010938; //0x015D
	bool N0001093B; //0x015E
	bool isEngineOn; //0x015F
	char pad_0160[40]; //0x0160
}; //Size: 0x0188

class HandbrakeComponent
{
public:
	EHandbrakeState handbrakeState; //0x0000
	char pad_0004[4]; //0x0004
	class N0001090C* N00002EE9; //0x0008
	__m128 timeSinceHandbraking; //0x0010
	bool isHandbrakeOn; //0x0020
	bool N00010905; //0x0021
	unsigned __int16 N00010908; //0x0022
	char pad_0024[12]; //0x0024
	__m128 timeForTurnComplete; //0x0030
	__m128 angVelDirection; //0x0040
	__m128 N00002F03; //0x0050
	class RaceVehicleDriftConfigData* driftConfig; //0x0060
	class RaceVehicleBrakesConfigData* brakesConfig; //0x0068
	class PerformanceModificationComponent* perfModComponent; //0x0070
}; //Size: 0x0078

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
	int64_t pad_0000[5];
	class RaceVehiclePerformanceModifierData* m_vehicleModifications;
	class RaceVehiclePerformanceModifierData* m_gearChangeTimeMods;
	int64_t pad_0038[48];
	ModifiedValueCache m_modifiedValueCache[159];
	PerformanceModifier m_modifiers[159];
}; static_assert(sizeof(PerformanceModificationComponent) == 0xE28, "PerfModComponent error");

class SteeringComponent
{
public:
	class NFSVehicle* nfsVehicle; //0x0000
	float steeringScale; //0x0008
	float wheelSteeringAngleRadians; //0x000C
}; //Size: 0x0010