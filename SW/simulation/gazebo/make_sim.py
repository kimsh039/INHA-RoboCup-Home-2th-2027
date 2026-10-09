#!/usr/bin/env python3
"""Generate the Gazebo world build/motion.world.sdf from robot_description/robocup.urdf.

The URDF is not modified on disk. For the simulation only, the wheel contacts are
replaced (see set_wheel_contacts), drive/arm/joint-state plugins are added and the
robot is placed in an empty world or the 6 x 6 m task room (one table in the middle).
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
# Task room: inner 6 x 6 m (x -1 .. 5, y -3 .. 3) with the table in the middle. The robot spawns at
# the world origin, 1 m from the west wall facing the table, so map/odom start on world axes.
# Walls 0.1 m thick, 1.0 m high; the partition breaks the symmetry so scan matching has a unique fit.
ROOM_CENTRE = (2.0, 0.0)
ROOM_WALLS = [('wall_north', (0, 3.05), '6.2 0.1 1.0'), ('wall_south', (0, -3.05), '6.2 0.1 1.0'),
              ('wall_east', (3.05, 0), '0.1 6.0 1.0'), ('wall_west', (-3.05, 0), '0.1 6.0 1.0'),
              ('wall_partition', (-1.5, 2.25), '0.1 1.5 1.0')]   # (x, y) from the room centre
# One table, the team's DESKER computer desk 2.0 W1600 x D800 (DSDBB1608, colour MLWW), from the
# maker's drawings (desker.co.kr/product/detail/615): top 28 mm E0 PB with LPM finish in maple,
# surface at 720 mm; the front (+y, north) edge has a 470 mm wide, 60 mm deep cable notch;
# powder-coated white steel legs at the corners, a 40 mm frame under the top (651 mm clear) and a
# 520 x 122 mm cable tray under the notch, 581 mm clear. Long side along x (x 1.2 .. 2.8, y -0.4 .. 0.4).
# Not on the drawings, so estimated from the product photos: 30 mm square legs, the frame on the
# short sides and the back, the tray hanging just behind the notch.
TABLE_CENTRES = [ROOM_CENTRE]
TABLE_SURFACE_Z = 0.72
TABLE = dict(length=1.6, depth=0.8, top=0.028, leg=0.03, frame=0.04, frame_t=0.02,
             notch_open=0.47, notch_bottom=0.32, notch_depth=0.06,
             tray_length=0.52, tray_depth=0.122, tray_bottom_z=0.581, tray_lip=0.02, sheet=0.002)
MAPLE = '0.80 0.69 0.55 1'
STEEL_WHITE = '0.93 0.93 0.92 1'
# Objects on the table (meshes in objects/, see objects/README.md): name -> (x, y, yaw, mass kg, collision).
# Task: pick the object a person asks for and place it on a free spot of the table. The six objects
# lie about 0.2 m in from the long sides, where the docked arm reaches; the middle of the south side
# is left free.
# Mesh origins sit on the supporting surface. Collision is a primitive around the mesh:
# ('box', centre xyz, size xyz) / ('sphere', centre xyz, radius) / ('cylinder', centre xyz, radius, length).
# Masses are of the real items (YCB fruit are light plastic replicas); the can holds 350 ml of soda.
OBJECTS = {
    'mug': (1.50, -0.20, 1.2, 0.118, ('box', (-0.0085, 0.0175, 0.040), (0.117, 0.093, 0.082))),
    'banana': (2.50, -0.22, 0.5, 0.120, ('box', (0.0115, -0.0075, 0.018), (0.109, 0.178, 0.036))),
    'fanta_can': (1.45, 0.20, 0.0, 0.377, ('cylinder', (0, 0, 0.061), 0.033, 0.122)),
    'peach': (1.85, 0.22, 0.0, 0.130, ('sphere', (-0.0143, 0.0056, 0.0293), 0.0305)),
    'apple': (2.15, 0.20, 0.0, 0.180, ('sphere', (0.001, -0.0035, 0.036), 0.036)),
    'green_apple': (2.50, 0.22, 0.0, 0.180, ('sphere', (0.001, -0.0035, 0.036), 0.036)),
}
OBJECT_DIR = HERE / 'objects'
GUI_PLUGINS = ('GzSceneManager', 'InteractiveViewControl', 'SelectEntities', 'CameraTracking', 'WorldControl',
               'WorldStats', 'EntityTree', 'TransformControl', 'JointPositionController')


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--test-wall', action='store_true', help='Add a wall 2 m forward for sensor validation')
    parser.add_argument('--world', choices=['empty', 'room'], default='empty',
                        help='room: walls, partition and one table with objects for the pick-and-place task (see README)')
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
    for name, (dx, dy), size in ROOM_WALLS:
        static_box(world, name, f'{ROOM_CENTRE[0] + dx:.3f} {ROOM_CENTRE[1] + dy:.3f} 0.5 0 0 0', size)
    for n, (tx, ty) in enumerate(TABLE_CENTRES, 1):
        add_table(world, f'table{n}', tx, ty)
    add_table_objects(world)


def table_front_edge(x):
    """y of the front edge at x (table frame): the notch is a flat bottom with cosine flanks."""
    half_open, half_bottom = TABLE['notch_open']/2, TABLE['notch_bottom']/2
    s, front = abs(x), TABLE['depth']/2
    if s >= half_open:
        return front
    if s <= half_bottom:
        return front - TABLE['notch_depth']
    u = (half_open - s)/(half_open - half_bottom)
    return front - TABLE['notch_depth']*(1 - math.cos(math.pi*u))/2


def write_table_top_mesh(path):
    """Top as an OBJ (surface at z=0): the outline is x-monotone, so it is a strip of quads."""
    half, back, z0, z1 = TABLE['length']/2, -TABLE['depth']/2, -TABLE['top'], 0.0
    ho, hb = TABLE['notch_open']/2, TABLE['notch_bottom']/2
    flank = [ho - (ho - hb)*i/12 for i in range(13)]
    xs = sorted({-half, half, *flank, *(-x for x in flank)})
    v, vn, f = [], [], []

    def quad(a, b, c, d, n):           # counter-clockwise seen from the normal side
        vn.append(n)
        f.append([(i, len(vn)) for i in (a, b, c, d)])

    for x in xs:
        y = table_front_edge(x)
        v += [(x, back, z0), (x, y, z0), (x, back, z1), (x, y, z1)]
    for i in range(len(xs) - 1):
        a, b = 4*i + 1, 4*i + 5                       # OBJ indices start at 1
        quad(a+2, b+2, b+3, a+3, (0, 0, 1))           # top
        quad(a, a+1, b+1, b, (0, 0, -1))              # bottom
        quad(a, b, b+2, a+2, (0, -1, 0))              # back
        dx, dy = xs[i+1] - xs[i], table_front_edge(xs[i+1]) - table_front_edge(xs[i])
        n = math.hypot(dx, dy)
        quad(b+1, a+1, a+3, b+3, (-dy/n, dx/n, 0))    # front, following the notch
    quad(1, 3, 4, 2, (-1, 0, 0))                      # left end
    e = 4*(len(xs) - 1) + 1
    quad(e, e+1, e+3, e+2, (1, 0, 0))                 # right end
    lines = [f'v {x:.5f} {y:.5f} {z:.5f}' for x, y, z in v] + [f'vn {x:.5f} {y:.5f} {z:.5f}' for x, y, z in vn]
    lines += ['f ' + ' '.join(f'{i}//{n}' for i, n in face) for face in f]
    path.write_text('\n'.join(lines) + '\n')


def add_table(world, name, tx, ty):
    """Static table: mesh top (box collisions around the notch), steel legs, frame and cable tray."""
    T = TABLE
    half, front, z = T['length']/2, T['depth']/2, TABLE_SURFACE_Z
    under = z - T['top']                                    # underside of the top
    model = sub(world, 'model', name=name)
    sub(model, 'static', 'true')
    sub(model, 'pose', f'{tx} {ty} 0 0 0 0')
    link = sub(model, 'link', name='link')
    BUILD.mkdir(exist_ok=True)
    mesh = BUILD / 'desker_desk_2_0_top.obj'                # Gazebo caches meshes by file name
    write_table_top_mesh(mesh)
    visual = sub(link, 'visual', name='top')
    sub(visual, 'pose', f'0 0 {z} 0 0 0')
    sub(sub(sub(visual, 'geometry'), 'mesh'), 'uri', mesh.as_uri())
    material = sub(visual, 'material')
    for key in ('ambient', 'diffuse'):
        sub(material, key, MAPLE)

    def box(part, centre, size, color, tags=('visual', 'collision')):
        for tag in tags:
            element = sub(link, tag, name=f'{part}_{tag}')
            sub(element, 'pose', '{:.4f} {:.4f} {:.4f} 0 0 0'.format(*centre))
            sub(sub(sub(element, 'geometry'), 'box'), 'size', '{:.4f} {:.4f} {:.4f}'.format(*size))
            if tag == 'visual':
                m = sub(element, 'material')
                for key in ('ambient', 'diffuse'):
                    sub(m, key, color)

    # Top collision: the full-depth part behind the notch plus the two front strips beside it.
    inner = front - T['notch_depth']
    # The flanks of the notch are left out (objects do not stand there).
    box('top_main', (0, (inner - front)/2, z - T['top']/2), (T['length'], inner + front, T['top']), MAPLE,
        ('collision',))
    strip = half - T['notch_open']/2
    for side, sx in (('left', -1), ('right', 1)):
        box(f'top_{side}', (sx*(half - strip/2), (inner + front)/2, z - T['top']/2), (strip, T['notch_depth'], T['top']),
            MAPLE, ('collision',))
    # Legs straight below the corners.
    leg = T['leg']
    for i, (sx, sy) in enumerate(((1, 1), (1, -1), (-1, 1), (-1, -1))):
        box(f'leg{i}', (sx*(half - leg/2), sy*(front - leg/2), under/2), (leg, leg, under), STEEL_WHITE)
    # Frame under the top: the two short sides and the back (the front stays open for the knees).
    fz, ft = under - T['frame']/2, T['frame_t']
    for side, sx in (('left', -1), ('right', 1)):
        box(f'frame_{side}', (sx*(half - ft/2), 0, fz), (ft, 2*front - 2*leg, T['frame']), STEEL_WHITE)
    box('frame_back', (0, -front + ft/2, fz), (2*half - 2*leg, ft, T['frame']), STEEL_WHITE)
    # Cable tray: back plate hanging from the top, a floor and a front lip, behind the notch.
    sh, lip_y = T['sheet'], inner - 0.005
    plate_y = lip_y - T['tray_depth']
    tz, length = T['tray_bottom_z'], T['tray_length']
    box('tray_back', (0, plate_y + sh/2, (tz + under)/2), (length, sh, under - tz), STEEL_WHITE)
    box('tray_floor', (0, (plate_y + lip_y)/2, tz + sh/2), (length, T['tray_depth'], sh), STEEL_WHITE)
    box('tray_lip', (0, lip_y - sh/2, tz + T['tray_lip']/2), (length, sh, T['tray_lip']), STEEL_WHITE)


def add_table_objects(world):
    for name, (x, y, yaw, mass, (shape, centre, *dims)) in OBJECTS.items():
        model = sub(world, 'model', name=name)
        sub(model, 'pose', f'{x} {y} {TABLE_SURFACE_Z + 0.002} 0 0 {yaw}')
        link = sub(model, 'link', name='link')
        if shape == 'box':
            sx, sy, sz = dims[0]
            diagonal = ((sy*sy + sz*sz)/12, (sx*sx + sz*sz)/12, (sx*sx + sy*sy)/12)
        elif shape == 'sphere':
            diagonal = (0.4*dims[0]**2,)*3
        else:
            r, length = dims
            diagonal = ((3*r*r + length*length)/12,)*2 + (r*r/2,)
        inertial = sub(link, 'inertial')
        sub(inertial, 'pose', '{} {} {} 0 0 0'.format(*centre))
        sub(inertial, 'mass', str(mass))
        inertia = sub(inertial, 'inertia')
        for key, value in zip(('ixx', 'iyy', 'izz'), diagonal):
            sub(inertia, key, f'{mass*value:.3e}')
        collision = sub(link, 'collision', name='collision')
        sub(collision, 'pose', '{} {} {} 0 0 0'.format(*centre))
        geometry = sub(sub(collision, 'geometry'), shape)
        if shape == 'box':
            sub(geometry, 'size', '{} {} {}'.format(*dims[0]))
        else:
            sub(geometry, 'radius', str(dims[0]))
            if shape == 'cylinder':
                sub(geometry, 'length', str(dims[1]))
        visual = sub(link, 'visual', name='visual')
        # Gazebo caches meshes and textures by file name, so each object has its own names.
        sub(sub(sub(visual, 'geometry'), 'mesh'), 'uri', (OBJECT_DIR / name / f'{name}.obj').as_uri())


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
    sub(view, 'camera_pose', '-0.6 -2.6 2.6 0 0.5 0.86')   # south-west corner, looking at robot and table
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
        sub(target, 'pose', '4.4 0.25 1.3 0 0 0')       # behind the table, in front of the east wall
        link = sub(target, 'link', name='board')
        visual = sub(link, 'visual', name='image')
        # A plane with normal -x maps the image's up axis to +y; roll it upright.
        sub(visual, 'pose', '0 0 0 1.5707963 0 0')
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
