#pragma once

#if !USE_REVIVAL_COMPONENT
namespace fb
{
#endif // !USE_REVIVAL_COMPONENT

	class DriftParams
	{
	public:

		class DriftConfigData
		{
		public:
			class RaceVehicleDriftConfigData* m_driftConfig;
			class PerformanceModificationComponent* m_performanceModificationComponent;
		};

		class DriftTriggerParams : public DriftConfigData
		{
			//public:
				//float GetMinSpeedToEnterDrift();
				//struct Vec4& GetSlipAngleToEnterDrift(struct Vec4& retVal);
				//struct Vec4& GetSlipAngleToEnterDriftAfterBrakeStab(struct Vec4& retVal);
				//float GetBrakeStabReturnSpeed();
				//struct Vec4& GetSlipAngleToEnterDriftWhenHandBraking(struct Vec4& retVal);
				//struct Vec4& GetSlipAngleToEnterDriftWhenBraking(struct Vec4& retVal);
				//struct Vec4& GetSlipAngleToEnterDriftAfterScandinavianFlick(struct Vec4& retVal);
		};

		class DriftScaleParams : public DriftConfigData
		{
		};

		class YawTorqueParams : public DriftConfigData
		{
		public:
			//struct Vec4 GetInitialYawTorque();
		};

		class SideForceParams : public DriftConfigData
		{
		};

		class OtherParams : public DriftConfigData
		{
			//public:
				//struct Vec4& GetProportionOfSpeedMaintainedAlongVelocity(struct Vec4& retVal);
				//struct Vec4& GetProportionOfSpeedMaintainedAlongZ(struct Vec4& retVal);
		};

		class ExitParams : public DriftConfigData
		{
		};

		class DonutParams : public DriftConfigData
		{
		};

		DriftTriggerParams mDriftTriggerParams;
		DriftScaleParams   mDriftScaleParams;
		YawTorqueParams    mYawTorqueParams;
		SideForceParams    mSideForceParams;
		OtherParams        mOtherParams;
		ExitParams         mExitParams;
		DonutParams        m_donutParams;
	};

#if !USE_REVIVAL_COMPONENT
}
#endif // !USE_REVIVAL_COMPONENT
