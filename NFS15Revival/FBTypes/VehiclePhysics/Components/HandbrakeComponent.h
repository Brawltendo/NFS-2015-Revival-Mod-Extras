#pragma once
#include <math/Vec4.h>


#if !USE_REVIVAL_COMPONENT
// forward declare here to keep out of the fb namespace
class RaceCarPhysicsObject;

namespace fb
{
#endif // !USE_REVIVAL_COMPONENT

enum EHandbrakeState
{
	EHandbrakeStateOff,
	EHandbrakeStateOnForwards,
	EHandbrakeStateTurning,
	EHandbrakeStateTurnComplete,
	EHandbrakeStateOnBackwards,
	EHandbrakeStateJTurnComplete,
};

class HandbrakeComponent
{
public:
	Vec4 GetDriftScaleFromHandbrake(const class RaceCarPhysicsObject& lpRaceCar);

	EHandbrakeState meState; //0x0000
	char pad_0004[12]; //0x0004
	Vec4 mvfHandbrakeTimer; //0x0010
	bool mbHandbrakeOn; //0x0020
	bool mbHandbrakeOnLastUpdate; //0x0021
	char pad_0022[14]; //0x0022
	Vec4 mvfLimitSteeringTimer; //0x0030
	Vec4 mvfHandbrakeTurnDirection; //0x0040
	Vec4 mSteeringLimits; //0x0050
	class RaceVehicleDriftConfigData* driftConfig; //0x0060
	class RaceVehicleBrakesConfigData* brakesConfig; //0x0068
	class PerformanceModificationComponent* perfModComponent; //0x0070
}; //Size: 0x0078

#if !USE_REVIVAL_COMPONENT
}
#endif // !USE_REVIVAL_COMPONENT