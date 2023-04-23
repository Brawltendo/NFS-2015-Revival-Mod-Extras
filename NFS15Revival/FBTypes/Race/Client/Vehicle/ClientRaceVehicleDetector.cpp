#include "ClientRaceVehicleDetector.h"
#include <util/utils.h>
#include <Engine/Entity/EntityIterator.h>


namespace fb
{

	uintptr_t g_clientGameWorld = 0x142B52688;
	const uintptr_t ClientRaceVehicleEntity_typeinfo = 0x1430226C0;
	const uintptr_t ClientRaceVehicleComponent_typeinfo = 0x143023580;


	const bool g_nearbyVehicleDetectorEnabled = true;

	class ComponentCollection
	{
	public:
		LinearTransform* getWorldTransforms();

	private:
		void* owner;
		uint8_t totalCount;
		uint8_t offsetCount;
		uint8_t transformCount;
	};

	class GenericEntity
	{
	public:
		void** __vftable;
		char pad[48];
		ComponentCollection* m_collection;
	};


	LinearTransform* ComponentCollection::getWorldTransforms()
	{
		uint8_t* data = reinterpret_cast<uint8_t*>(this + 1);
		uint8_t* localTransforms = data + (32 * offsetCount);
		LinearTransform* worldTransforms = reinterpret_cast<LinearTransform*>(localTransforms + sizeof(LinearTransform) * transformCount);
		return worldTransforms;
	}

	float pointToLineDistance(const Vec4* pos, const Vec4* v0, const Vec4* v1, Vec4& closestPoint)
	{
		float(__fastcall* native)(const Vec4*, const Vec4*, const Vec4*, Vec4&) = reinterpret_cast<float(__fastcall*)(const Vec4*, const Vec4*, const Vec4*, Vec4&)>(0x143B07E50);
		return native(pos, v0, v1, closestPoint);
	}

	float ClientRaceVehicleDetector::update(const LinearTransform& transform, float deltaTime, float speedThreshold, const GenericEntity* excludedVehicle)
	{
		// native functions
		typedef uintptr_t(__fastcall* getComponentOfType)(GenericEntity*, uintptr_t);
		Vec4&(__fastcall* linearVelocity)(uintptr_t, Vec4&) = reinterpret_cast<Vec4&(__fastcall*)(uintptr_t, Vec4&)>(0x1441A41C0);

		// min speed difference (in MPH) between 2 cars to trigger camera shake
		const float NearMissShakeSpeedDifference = 50.f;
		float smallestDistanceToOtherVehicle = Infinity;

		if (g_nearbyVehicleDetectorEnabled)
		{
			TypedEntityIterator<ClientRaceVehicleEntity_typeinfo> allVehicles(*(uintptr_t*)g_clientGameWorld);
			while (allVehicles.link())
			{
				GenericEntity* vehicle = reinterpret_cast<GenericEntity*>(allVehicles.next(0x40));
				if (!vehicle)
					break;

				if (VecDistanceBetween(vehicle->m_collection->getWorldTransforms()[0].trans, transform.trans).x <= m_squaredRadius && vehicle != excludedVehicle)
				{
					uintptr_t vehicleComponent = getComponentOfType(vehicle->__vftable[33])(vehicle, ClientRaceVehicleComponent_typeinfo);
					Vec4 velocity;
					linearVelocity(vehicleComponent, velocity);
					const LinearTransform& vehicleTransform = vehicle->m_collection->getWorldTransforms()[0];
					const bool areVehiclesFacingSameDir = Dot3(transform.forward, vehicleTransform.forward) > 0.f;
					const float vehicleSpeed = fabsf(Dot3(velocity, vehicleTransform.forward));
					// the function that calls this one adds 25 to the threshold, but we just want the base speed
					const float baseSpeed = speedThreshold - 25.f;
					const float relativeSpeed = areVehiclesFacingSameDir ? (baseSpeed - vehicleSpeed) : (baseSpeed + vehicleSpeed);
					if (fabsf(relativeSpeed) >= NearMissShakeSpeedDifference * 0.44704f)
					{
						Vec4 closestPoint;
						const Vec4 vehiclePosition = vehicleTransform.trans;
						const Vec4 estimatedPreviousVehiclePosition = VecSub(vehiclePosition, VecMul(deltaTime, velocity));
						const float distanceToOtherVehicle = pointToLineDistance(&transform.trans, &estimatedPreviousVehiclePosition, &vehiclePosition, closestPoint);
						smallestDistanceToOtherVehicle = fminf(distanceToOtherVehicle, smallestDistanceToOtherVehicle);
					}
				}
			}
			return smallestDistanceToOtherVehicle;
		}
		
	}

}