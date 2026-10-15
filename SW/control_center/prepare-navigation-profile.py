#!/usr/bin/env python3
"""Generate Nav2's footprint from the exact URDF used by the 3D guard."""
import argparse
import copy
import math
from pathlib import Path
import sys

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent.parent/'robot'))
from collision_geometry import RobotGeometry, convex_footprint


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--input', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--safety', required=True)
    args = parser.parse_args()
    safety = yaml.safe_load(Path(args.safety).read_text())['/**']['ros__parameters']
    profile = yaml.safe_load(Path(args.input).read_text())
    model = RobotGeometry()
    control = profile['controller_server']['ros__parameters']
    # Ordinary path following uses forward motion and in-place turns. Reverse
    # belongs to recovery only, after both turn directions are blocked.
    control['FollowPath'].update(min_vel_x=0.,
                                 min_speed_xy=0., min_speed_theta=0., xy_goal_tolerance=.05,
                                 trans_stopped_velocity=.01)
    # Global inflation shapes a path through the corridor's low-cost middle.
    # Keep the local obstacle weight below the goal attraction so an explicitly
    # requested endpoint near furniture remains reachable.
    control['FollowPath']['BaseObstacle.scale'] = .03
    checker = control.get('progress_checker_plugin', 'progress_checker')
    control['progress_checker_plugin'] = checker
    control[checker] = dict(plugin='nav2_controller::PoseProgressChecker', required_movement_radius=.05,
                           required_movement_angle=.10, movement_time_allowance=6.)
    if 'general_goal_checker' not in control['goal_checker_plugins']:
        control['goal_checker_plugins'].append('general_goal_checker')
    control['general_goal_checker'] = dict(plugin='nav2_controller::StoppedGoalChecker', stateful=False,
                                           xy_goal_tolerance=.05, yaw_goal_tolerance=.02,
                                           trans_stopped_velocity=.01, rot_stopped_velocity=.02)
    # A tabletop and the mast occupy different heights. The full 3D swept
    # geometry remains enforced by the guard; its XY envelope must not reject
    # every table-approach trajectory at the starting pose. Keep Nav2's inflated
    # center obstacle critic and path critics for coarse obstacle avoidance.
    control['GraspApproach'] = copy.deepcopy(control['FollowPath'])
    control['GraspApproach']['critics'] = [critic for critic in control['GraspApproach']['critics']
                                           if critic != 'ObstacleFootprint']
    control['GraspApproach']['BaseObstacle.scale'] = .01
    if 'GraspApproach' not in control['controller_plugins']:
        control['controller_plugins'].append('GraspApproach')
    if 'grasp_stage_checker' not in control['goal_checker_plugins']:
        control['goal_checker_plugins'].append('grasp_stage_checker')
    control['grasp_stage_checker'] = dict(plugin='nav2_controller::SimpleGoalChecker', stateful=True,
                                           xy_goal_tolerance=.10, yaw_goal_tolerance=.08)
    planner = profile['planner_server']['ros__parameters']
    planner['DetectionTarget'] = copy.deepcopy(planner['GridBased'])
    # The object XY can be occupied by the object/table in a 2D map. Plan to
    # the nearest reachable free endpoint, then continue under the 3D guard.
    # This is a coarse planner search tolerance, not a final stand-off distance.
    planner['DetectionTarget']['tolerance'] = 1.0
    if 'DetectionTarget' not in planner['planner_plugins']:
        planner['planner_plugins'].append('DetectionTarget')
    # A table-side staging goal must reach that side. A broad nearest-free
    # tolerance can otherwise finish back on the robot's original side.
    planner['TableStage'] = copy.deepcopy(planner['GridBased'])
    planner['TableStage']['tolerance'] = .05
    if 'TableStage' not in planner['planner_plugins']:
        planner['planner_plugins'].append('TableStage')
    profile['bt_navigator']['ros__parameters']['default_nav_to_pose_bt_xml'] = str(
        Path(__file__).resolve().with_name('navigate-to-pose-replanning.xml'))
    # The guard updates this with measured arm geometry after startup. Start
    # with the static URDF shape and fail closed until arm feedback is available.
    outline = convex_footprint(model.fixed)
    for kind in ('local_costmap', 'global_costmap'):
        settings = profile[kind][kind]['ros__parameters']
        # The 3D guard enforces clearance along the actual motion. Expanding
        # every side here would reject moving away from a close rear obstacle.
        settings['footprint'] = str(outline)
        settings['footprint_padding'] = -0.05  # 각 변 5 cm 줄임 (실제 차체 보호는 3D 충돌 관문이 실제 크기로 유지)
        if safety.get('body_only', False) and 'inflation_layer' in settings:
            # Retain the configured soft preference field. Clamping inflation
            # to the body radius makes walls and the corridor middle cost the
            # same immediately beyond the physical collision boundary.
            radius = min(abs(a[0]*b[1]-a[1]*b[0])/math.hypot(b[0]-a[0], b[1]-a[1])
                         for a, b in zip(outline, outline[1:]+outline[:1]))
            # Polygon transport is float32. A micrometre rounding allowance
            # avoids a continuous "smaller than inscribed radius" error flood.
            minimum_radius = math.ceil(radius*1e6)/1e6+1e-6
            inflation = settings['inflation_layer']
            inflation['inflation_radius'] = max(1.5, float(inflation.get('inflation_radius', 1.5)), minimum_radius)
            inflation['cost_scaling_factor'] = min(2., float(inflation.get('cost_scaling_factor', 2.)))
        for layer in ('voxel_layer', 'obstacle_layer', 'static_layer'):
            if layer in settings:
                settings[layer]['footprint_clearing_enabled'] = False
        for layer in ('voxel_layer', 'obstacle_layer'):
            if layer in settings:
                # MID-360 memory belongs to the cloud processor. Do not build
                # another marking/clearing memory with different input here.
                sources = settings[layer].get('observation_sources', '').split()
                settings[layer]['observation_sources'] = ' '.join(
                    source for source in sources if source not in ('mid360', 'mid360_clear'))
                for source in ('mid360', 'mid360_clear'):
                    settings[layer].pop(source, None)
        plugins = settings['plugins']
        if 'collision_scene' not in plugins:
            plugins.insert(plugins.index('inflation_layer') if 'inflation_layer' in plugins else len(plugins),
                           'collision_scene')
        settings['collision_scene'] = dict(
            plugin='robocup_collision_processing::SceneLayer', enabled=True,
            topic=safety['safety_topic'], min_height_above_floor=float(safety['floor_band']),
            max_obstacle_height=float(safety['max_obstacle_height']),
            voxel_size=float(safety['safety_voxel']), cloud_timeout=float(safety['cloud_timeout']),
            use_source_points=bool(safety.get('body_only', False)),
            maximum_cells=int(safety['maximum_cells']))
    Path(args.output).write_text(yaml.safe_dump(profile, sort_keys=False))


if __name__ == '__main__':
    main()
