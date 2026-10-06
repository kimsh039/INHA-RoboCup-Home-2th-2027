#!/usr/bin/env python3
"""Generate the Gazebo world build/motion.world.sdf from robot_description/robocup.urdf.

The URDF is not modified on disk. For the simulation only, the wheel contacts are
replaced (see set_wheel_contacts), drive/arm/joint-state plugins are added and the
robot is placed in an empty world or the 6 x 6 m test room.
"""
import argparse
import math
import subprocess
import tempfile
import xml.etree.ElementTree as E
from pathlib import Path

HERE = Path(__file__).resolve().parent
DESCRIPTION = HERE.parent / 'robot_description'
ROBOT_URDF = DESCRIPTION / 'robocup.urdf'
BUILD = HERE / 'build'
WORLD_FILE = BUILD / 'motion.world.sdf'

WORLD_NAME = 'robocup_motion'
MODEL_NAME = 'robocup'
SPAWN_POSE = '0 0 0.145 0 0 0'          # wheels just above the floor

# Tracer drive (DiffDrive): wheel geometry from the URDF, speed from the TRACER datasheet,
# accelerations assumed (an instant step makes the wheels slip).
DIFF_DRIVE = dict(
    left_joint='left_wheel', right_joint='right_wheel', wheel_separation='0.34', wheel_radius='0.0605',
    topic='/robocup/cmd_vel', odom_topic='/robocup/odometry', odom_publish_frequency='10',
    max_linear_velocity='1.6', min_linear_velocity='-1.6',
    max_linear_acceleration='1.0', min_linear_acceleration='-1.0',
    max_angular_acceleration='2.0', min_angular_acceleration='-2.0',
    frame_id='odom', child_frame_id='base_link', tf_topic='/model/robocup/tf')
ARM_CMD_MAX = '0.5'                      # rad/s, Piper joints
GRIPPER_CMD_MAX = '0.02'                 # m/s, gripper fingers
JOINT_STATES_TOPIC = '/robocup/joint_states'
# Mid-360S scan grid, replacing the URDF's 1000 x 20. Gazebo repeats one fixed grid every frame,
# while the real non-repetitive pattern fills its FOV over time; 20 rings left only 2-3 lines on a
# table top. 60 rings (1.0 deg apart) x 340 keeps the datasheet's 200,000 points/s at 10 Hz.
MID360_SCAN = dict(horizontal=340, vertical=60)

WALL_GREY = '0.75 0.75 0.75 1'
# Test room: inner 6 x 6 m centred on the spawn point; walls 0.1 m thick, 1.0 m high.
# The partition breaks the square's symmetry so scan matching has a unique fit.
ROOM_WALLS = [('wall_north', '0 3.05 0.5 0 0 0', '6.2 0.1 1.0'), ('wall_south', '0 -3.05 0.5 0 0 0', '6.2 0.1 1.0'),
              ('wall_east', '3.05 0 0.5 0 0 0', '0.1 6.0 1.0'), ('wall_west', '-3.05 0 0.5 0 0 0', '0.1 6.0 1.0'),
              ('wall_partition', '-1.5 2.25 0.5 0 0 0', '0.1 1.5 1.0')]
# Table 1.6 x 0.8 m, top 0.03 m thick with surface at 0.72 m; legs 0.05 m square inset 0.05 m.
TABLE_CENTRE = (1.8, -1.0)
TABLE_LEG_OFFSETS = [(.725, .325), (.725, -.325), (-.725, .325), (-.725, -.325)]
GUI_PLUGINS = ('GzSceneManager', 'InteractiveViewControl', 'SelectEntities', 'CameraTracking', 'WorldControl',
               'WorldStats', 'EntityTree', 'TransformControl', 'JointPositionController')


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--test-wall', action='store_true', help='Add a wall 2 m forward for sensor validation')
    parser.add_argument('--world', choices=['empty', 'room'], default='empty',
                        help='room: walls, partition and table for SLAM/Nav2 tests (see README)')
    parser.add_argument('--detection-demo', action='store_true', help='640x480 head sensors, 10 Hz, 5 m depth')
    parser.add_argument('--target-image', type=Path, help='Add a textured test billboard ahead of the robot')
    return parser.parse_args()


def sub(parent, tag, text=None, **attrib):
    element = E.SubElement(parent, tag, attrib)
    if text is not None:
        element.text = str(text)
    return element


def plugin(parent, filename, name, **params):
    p = sub(parent, 'plugin', filename=filename, name=name)
    for key, value in params.items():
        sub(p, key, value)
    return p


def load_robot_urdf():
    """robocup.urdf with mesh paths made absolute (the SDF is written elsewhere)."""
    urdf = E.parse(ROBOT_URDF)
    for mesh in urdf.findall('.//mesh'):
        mesh.set('filename', str((DESCRIPTION / mesh.get('filename')).resolve()))
    return urdf


def set_mid360_scan(urdf):
    """Denser vertical sampling for the sim Mid-360S (see MID360_SCAN); FOV and range unchanged."""
    scan = urdf.find(".//sensor[@name='livox_mid360s']/lidar/scan")
    for axis, samples in MID360_SCAN.items():
        element = scan.find(axis)
        element.find('samples').text = str(samples)
    horizontal = scan.find('horizontal')
    # Full circle without a duplicate ray at +pi.
    horizontal.find('max_angle').text = repr(math.pi - 2*math.pi/MID360_SCAN['horizontal'])


def set_wheel_contacts(urdf):
    """Sim-only wheel contacts; masses/inertias stay as in the URDF (Tracer 30 kg).

    With the source mesh collisions all six wheels touch the floor at once, so the drive
    wheels slipped against the casters (in-place turns lost ~20-30% of the commanded yaw).
    """
    def set_collision(link, shape, **size):
        c = urdf.find(f"link[@name='{link}']/collision")
        c.find('origin').attrib.update(xyz='0 0 0', rpy='0 0 0')
        g = c.find('geometry')
        g.clear()
        sub(g, shape, **{k: str(v) for k, v in size.items()})

    def friction(link, mu):
        g = sub(urdf.getroot(), 'gazebo', reference=link)
        for key in ('mu1', 'mu2'):
            sub(g, key, mu)

    for side in ('left', 'right'):
        # Sphere of the 60.5 mm wheel radius: one smooth contact point on the tyre centre line.
        # The faceted mesh and a full-width cylinder both scrubbed during in-place turns.
        set_collision(f'{side}_wheel_link', 'sphere', radius=0.0605)
        friction(f'{side}_wheel_link', 1.0)
    for corner in ('fl', 'fr', 'rl', 'rr'):
        # 1 mm smaller than the 37.5 mm caster wheel so the drive wheels carry the load (the real
        # drive wheels are sprung); low-friction casters only catch the body when it pitches.
        set_collision(f'{corner}_wheel_link', 'sphere', radius=0.0365)
        friction(f'{corner}_wheel_link', 0.01)


def urdf_to_model(urdf):
    """Convert with `gz sdf` through a temporary file; robocup.urdf stays the only stored URDF."""
    with tempfile.TemporaryDirectory(prefix='robocup-sdf-') as temporary:
        local = Path(temporary) / 'robot.urdf'
        urdf.write(local, encoding='unicode')
        result = subprocess.run(['gz', 'sdf', '-p', str(local)], capture_output=True, text=True, check=True)
    model = E.fromstring(result.stdout).find('model')
    model.set('name', MODEL_NAME)
    sub(model, 'pose', SPAWN_POSE)
    # Cancel the source left wheel joint's half-turn so both drive axes point +Y.
    model.find("joint[@name='left_wheel']/axis/xyz").text = '0 -1 0'
    return model


def add_robot_plugins(model):
    controller = ('gz-sim-joint-position-controller-system', 'gz::sim::systems::JointPositionController')
    for i in range(1, 7):
        plugin(model, *controller, joint_name=f'piper_joint{i}', use_velocity_commands='true', cmd_max=ARM_CMD_MAX)
    for i in (1, 2):
        plugin(model, *controller, joint_name=f'piper_gripper_joint{i}', use_velocity_commands='true',
               cmd_max=GRIPPER_CMD_MAX)
    plugin(model, 'gz-sim-joint-state-publisher-system', 'gz::sim::systems::JointStatePublisher',
           topic=JOINT_STATES_TOPIC)
    plugin(model, 'gz-sim-diff-drive-system', 'gz::sim::systems::DiffDrive', **DIFF_DRIVE)


def static_box(world, name, pose, size, color=WALL_GREY):
    model = sub(world, 'model', name=name)
    sub(model, 'static', 'true')
    sub(model, 'pose', pose)
    link = sub(model, 'link', name='link')
    for tag in ('visual', 'collision'):
        element = sub(link, tag, name=tag)
        sub(sub(sub(element, 'geometry'), 'box'), 'size', size)
        if tag == 'visual':
            material = sub(element, 'material')
            sub(material, 'ambient', color)
            sub(material, 'diffuse', color)


def add_ground(world):
    ground = sub(world, 'model', name='ground')
    sub(ground, 'static', 'true')
    link = sub(ground, 'link', name='ground')
    for tag in ('collision', 'visual'):
        plane = sub(sub(sub(link, tag, name='ground_' + tag), 'geometry'), 'plane')
        sub(plane, 'normal', '0 0 1')
        sub(plane, 'size', '100 100')


def add_test_wall(world):
    wall = sub(world, 'model', name='sensor_test_wall')
    sub(wall, 'static', 'true')
    sub(wall, 'pose', '2 0 1 0 0 0')
    link = sub(wall, 'link', name='wall')
    for tag in ('visual', 'collision'):
        sub(sub(sub(sub(link, tag, name='wall_' + tag), 'geometry'), 'box'), 'size', '0.2 4 2')


def add_room(world):
    for name, pose, size in ROOM_WALLS:
        static_box(world, name, pose, size)
    tx, ty = TABLE_CENTRE
    static_box(world, 'table_top', f'{tx} {ty} 0.705 0 0 0', '1.6 0.8 0.03', '0.55 0.35 0.2 1')
    for i, (dx, dy) in enumerate(TABLE_LEG_OFFSETS):
        static_box(world, f'table_leg{i}', f'{tx + dx} {ty + dy} 0.345 0 0 0', '0.05 0.05 0.69', '0.3 0.3 0.3 1')


def add_lighting_and_gui(world):
    light = sub(world, 'light', name='sun', type='directional')
    sub(light, 'pose', '0 0 5 0 0 0')
    sub(light, 'direction', '-0.5 -0.5 -1')
    sub(light, 'diffuse', '0.8 0.8 0.8 1')
    sub(sub(world, 'scene'), 'ambient', '0.6 0.6 0.6 1')
    gui = sub(world, 'gui', fullscreen='false')
    view = sub(gui, 'plugin', filename='MinimalScene', name='3D View')
    sub(view, 'engine', 'ogre2')
    sub(view, 'scene', 'scene')
    sub(view, 'camera_pose', '2.2 2.2 1.9 0 0.32 -2.35619')
    for name in GUI_PLUGINS:
        sub(gui, 'plugin', filename=name, name=name)


def build_world(model, room=False, test_wall=False):
    sdf = E.Element('sdf', version='1.10')
    world = sub(sdf, 'world', name=WORLD_NAME)
    sub(world, 'gravity', '0 0 -9.81')
    physics = sub(world, 'physics', name='physics', type='ignored')
    sub(physics, 'max_step_size', '0.001')
    sub(physics, 'real_time_factor', '1')
    for system, name in [('physics', 'Physics'), ('user-commands', 'UserCommands'),
                         ('scene-broadcaster', 'SceneBroadcaster')]:
        plugin(world, f'gz-sim-{system}-system', f'gz::sim::systems::{name}')
    plugin(world, 'gz-sim-sensors-system', 'gz::sim::systems::Sensors', render_engine='ogre2')
    world.append(model)
    add_ground(world)
    if test_wall:
        add_test_wall(world)
    if room:
        add_room(world)
    add_lighting_and_gui(world)
    return sdf


def main():
    args = parse_args()
    BUILD.mkdir(exist_ok=True)
    urdf = load_robot_urdf()
    if args.detection_demo:
        for name in ('d435f_color', 'd435f_depth'):
            sensor = urdf.find(f".//sensor[@name='{name}']")
            sensor.find('update_rate').text = '10'
            sensor.find('camera/image/width').text = '640'
            sensor.find('camera/image/height').text = '480'
            if name == 'd435f_depth':
                sensor.find('camera/clip/far').text = '5'
    set_mid360_scan(urdf)
    set_wheel_contacts(urdf)
    model = urdf_to_model(urdf)
    add_robot_plugins(model)
    sdf = build_world(model, room=args.world == 'room', test_wall=args.test_wall)
    if args.target_image:
        image = args.target_image.resolve(strict=True)
        world = sdf.find('world')
        target = sub(world, 'model', name='detection_billboard')
        sub(target, 'static', 'true')
        sub(target, 'pose', '2.2 0.25 1.3 0 0 0')
        link = sub(target, 'link', name='board')
        visual = sub(link, 'visual', name='image')
        plane = sub(sub(visual, 'geometry'), 'plane')
        sub(plane, 'normal', '-1 0 0')
        sub(plane, 'size', '1.5 2.0')
        material = sub(visual, 'material')
        sub(material, 'diffuse', '1 1 1 1')
        sub(material, 'ambient', '1 1 1 1')
        metal = sub(sub(material, 'pbr'), 'metal')
        sub(metal, 'albedo_map', image.as_uri())
        sub(metal, 'metalness', '0')
        sub(metal, 'roughness', '1')
        collision = sub(link, 'collision', name='board_collision')
        sub(sub(sub(collision, 'geometry'), 'box'), 'size', '0.04 1.5 2.0')
    E.indent(sdf)
    E.ElementTree(sdf).write(WORLD_FILE, encoding='unicode', xml_declaration=True)
    print(WORLD_FILE)


if __name__ == '__main__':
    main()
