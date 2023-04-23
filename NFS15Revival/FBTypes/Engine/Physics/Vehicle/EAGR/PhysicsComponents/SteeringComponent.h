#pragma once


#if !USE_REVIVAL_COMPONENT
namespace fb
{
#endif // !USE_REVIVAL_COMPONENT

class SteeringComponent
{
public:
	class NFSVehicle* nfsVehicle; //0x0000
	float steeringScale; //0x0008
	float wheelSteeringAngleRadians; //0x000C
}; //Size: 0x0010

#if !USE_REVIVAL_COMPONENT
}
#endif // !USE_REVIVAL_COMPONENT