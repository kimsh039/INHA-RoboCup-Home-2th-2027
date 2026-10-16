"""Whitelisted heuristic overlay, distinct from physical calibration."""
import copy
from .core import require
KEYS={'coarse_xy_error_bound_m','observation_image_margin_fraction','observation_max_distance_m',
      'wrist_search_offsets_m','wrist_search_radius_m','grasp_min_width_m','grasp_open_margin_m',
      'hold_min_effort','hold_width_slip_m','gripper_stall_m','observation_camera_quaternion','observation_surface_clearance_m','observation_camera_distance_m','observation_joints',
      'return_joints','topdown_quaternion','pregrasp_m','lift_m','retreat_m','place_offset_m',
      'velocity_scale','acceleration_scale','position_tolerance_m','orientation_tolerance_rad',
      'association_radius_m','association_margin_m','min_confidence','max_position_std_m','max_observation_age_s',
      'depth_min_m','depth_max_m','depth_min_pixels','depth_min_fraction','depth_max_mad_m',
      'mask_erosion_px','depth_patch_radius_px','observation_stable_frames','observation_retries',
      'stage_timeout_s','approach_timeout_s','observation_timeout_s','gripper_tolerance_m',
      'gripper_command_effort','grasp_min_effort','gripper_settle_s','driver_speed_percent',
      'max_joint_speed_rad_s','max_joint_acceleration_rad_s2','path_tolerance_rad','goal_tolerance_rad',
      'joint_stable_s','stable_speed_rad_s','support_margin_m','coarse_collision_padding_m','axis_stability_tolerance_rad'}
CLASS_KEYS={'yaw_sample_degrees','open_m','close_m','grip_height_m','closing_axis_yaw_offset_rad','enabled'}

def overlay(config, tuning):
    require(isinstance(tuning,dict),'INVALID_TUNING_FILE'); result=copy.deepcopy(config)
    require(set(tuning).issubset(KEYS|{'classes'}),'NON_HEURISTIC_TUNING_KEY')
    for key,value in tuning.items():
        if key!='classes':result[key]=value
        else:
            require(isinstance(value,dict),'INVALID_CLASS_TUNING')
            for name,values in value.items():
                require(name in result['classes'] and isinstance(values,dict) and set(values).issubset(CLASS_KEYS),'CLASS_TUNING_KEY')
                result['classes'][name].update(values)
    return result
