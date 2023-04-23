#pragma once
#include <math/LinearTransform.h>


namespace fb
{

	class GenericEntity;

	class ClientRaceVehicleDetector
	{
	public:
		float update(const LinearTransform& transform, float deltaTime, float speedThreshold, const GenericEntity* excludedVehicle);

	private:
		float m_squaredRadius;
		bool m_enabled;
	};

	

}