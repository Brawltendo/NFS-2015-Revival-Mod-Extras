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
