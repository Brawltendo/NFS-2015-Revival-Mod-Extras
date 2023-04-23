#pragma once
#include <xmmintrin.h>


#if !USE_REVIVAL_COMPONENT
namespace fb
{
#endif // !USE_REVIVAL_COMPONENT

class RaceVehicleDriftConfigData
{
public:
	char pad_0000[16]; //0x0000
	char* Name; //0x0010
	float Minimum_speed_to_enter_drift; //0x0018
	float Slip_angle_to_enter_drift; //0x001C
	float Slip_angle_to_enter_drift_when_handbraking; //0x0020
	float Slip_angle_to_enter_drift_when_braking; //0x0024
	float Slip_angle_to_enter_drift_after_brake_stab; //0x0028
	float Slip_angle_to_enter_drift_after_scandinavian_flick; //0x002C
	float Brake_stab_return_speed; //0x0030
	float Drift_angle_to_exit_drift; //0x0034
	float Steering_amount_on_exit_drift; //0x0038
	float Slip_angle_to_start_blending_down_drift_scale; //0x003C
	float Slip_angle_for_zero_drift_scale; //0x0040
	float Time_to_blend_damping; //0x0044
	float Starting_drift_scale; //0x0048
	float Starting_drift_scale_after_scandinavian_flick; //0x004C
	float Drift_scale_from_handbrake; //0x0050
	float Drift_scale_decay; //0x0054
	float Drift_scale_from_steering; //0x0058
	float Drift_scale_from_braking; //0x005C
	float Drift_scale_from_counter_steering; //0x0060
	float Drift_scale_from_gas_let_off; //0x0064
	float Drift_scale_from_gas_stab; //0x0068
	float Drift_angle_for_full_gas_influence; //0x006C
	float DriftScaleFromHandbrakeAtLowSpeed; //0x0070
	float DriftScaleFromHandbrakeAtHighSpeed; //0x0074
	float Initial_yaw_torque; //0x0078
	float Minimum_yaw_torque; //0x007C
	float Mid_yaw_torque; //0x0080
	float Maximum_yaw_torque; //0x0084
	float Gas_let_off_yaw_torque; //0x0088
	float Donut_yaw_torque; //0x008C
	float Donut_speed_limit_low; //0x0090
	float Donut_speed_limit_high; //0x0094
	float Default_steering_remapping; //0x0098
	float Counter_steering_remapping; //0x009C
	float Side_force_magnitude; //0x00A0
	float Drift_scale_for_maximum_side_force; //0x00A4
	float Speed_for_no_sideforce; //0x00A8
	float Speed_for_maximum_side_force; //0x00AC
	float Minimum_drift_angle_for_side_force; //0x00B0
	float Drift_angle_for_side_force; //0x00B4
	float Drift_angle_for_decay; //0x00B8
	float Decay_rate; //0x00BC
	float Drift_sideways_damping; //0x00C0
	float Min_decay_side_force_magnitude; //0x00C4
	float Side_force_multiplier; //0x00C8
	float Drift_angular_damping; //0x00CC
	float Maintain_entry_speed_amount; //0x00D0
	float Max_speed_difference_for_maintain_speed; //0x00D4
	float Reduce_forward_speed_amount; //0x00D8
	float Max_countersteering; //0x00DC
	float Default_steering; //0x00E0
	float Max_countersteering_at_low_slip_angle; //0x00E4
	float Max_countersteering_at_high_slip_angle; //0x00E8
	float Low_slip_angle_for_counter_steering; //0x00EC
	float High_slip_angle_for_counter_steering; //0x00F0
	float Slip_angle_for_deep_drift; //0x00F4
	float Max_counter_steering_power; //0x00F8
	float Slip_angle_for_full_damping_fade; //0x00FC
	float ExtraFishTailTorque; //0x0100
	char pad_0104[4]; //0x0104
	__m128 (*YawDampeningAtAngle)[10]; //0x0108
	__m128 (*YawDampeningAtSpeed)[10]; //0x0110
	float TimeForMaxDonutGripEffect; //0x0118
	float DonutLongitudinalGripMin; //0x011C
	float DonutLongitudinalGripMax; //0x0120
	float DonutLateralGripMin; //0x0124
	float DonutLateralGripMax; //0x0128
}; //Size: 0x012C

#if !USE_REVIVAL_COMPONENT
}
#endif // !USE_REVIVAL_COMPONENT