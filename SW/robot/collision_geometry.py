"""Conservative collision boxes from URDF, shared by Nav2 and the 3D guard.

Meshes become oriented bounding boxes; rack collision boxes stay separate.
All distances are to robot surfaces, in metres, never to the LiDAR origin.
"""
from functools import lru_cache
from itertools import product
from pathlib import Path
import struct
import xml.etree.ElementTree as ET

import numpy as np

REPO = Path(__file__).resolve().parents[2]
CORNERS = np.array(list(product((-1., 1.), repeat=3)))


def rpy_matrix(values):
    r, p, y = values
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                     [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
                     [-sp, cp*sr, cp*cr]])


def origin(element):
    out = np.eye(4)
    if element is not None:
        out[:3, :3] = rpy_matrix([float(v) for v in element.get('rpy', '0 0 0').split()])
        out[:3, 3] = [float(v) for v in element.get('xyz', '0 0 0').split()]
    return out


@lru_cache(maxsize=64)
def mesh_bounds(filename):
    path = Path(filename)
    if path.suffix.lower() == '.dae':
        root = ET.parse(path).getroot()
        ns = {'c': root.tag.split('}')[0].lstrip('{')}
        points = []
        for mesh in root.findall('c:library_geometries/c:geometry/c:mesh', ns):
            for vertex in mesh.findall('c:vertices/c:input', ns):
                if vertex.get('semantic') != 'POSITION':
                    continue
                source = mesh.find("c:source[@id='%s']" % vertex.get('source').lstrip('#'), ns)
                accessor = source.find('c:technique_common/c:accessor', ns)
                data = np.fromstring(source.find('c:float_array', ns).text, sep=' ')
                stride = int(accessor.get('stride', '3'))
                offset = int(accessor.get('offset', '0'))
                points.append(data[offset:offset + int(accessor.get('count'))*stride].reshape(-1, stride)[:, :3])
        # The committed Tracer assets contain identity scene nodes. Refuse other
        # assets rather than silently discarding a scene transform or axis change.
        for node in root.findall('c:library_visual_scenes/c:visual_scene//c:node', ns):
            if any(child.tag.rsplit('}', 1)[-1] in ('matrix', 'translate', 'rotate', 'scale') for child in node):
                raise ValueError('Collision DAE requires baking scene transforms: ' + filename)
        unit = root.find('c:asset/c:unit', ns)
        points = np.concatenate(points) * (float(unit.get('meter', '1')) if unit is not None else 1.)
    elif path.suffix.lower() == '.stl':
        data = path.read_bytes()
        count = struct.unpack_from('<I', data, 80)[0] if len(data) >= 84 else 0
        if len(data) == 84 + count * 50:
            dtype = np.dtype([('normal', '<f4', 3), ('vertices', '<f4', (3, 3)), ('attr', '<u2')])
            points = np.frombuffer(data, dtype=dtype, count=count, offset=84)['vertices'].reshape(-1, 3)
        else:
            points = np.array([[float(v) for v in line.split()[1:4]]
                               for line in data.decode('ascii').splitlines()
                               if line.strip().startswith('vertex ')])
    else:
        raise ValueError('Unsupported collision mesh: ' + filename)
    if not len(points) or not np.isfinite(points).all():
        raise ValueError('Empty or invalid collision mesh: ' + filename)
    return points.min(axis=0), points.max(axis=0)


class Box:
    def __init__(self, name, transform, half):
        self.name = name
        self.transform = np.asarray(transform, dtype=float)
        self.half = np.asarray(half, dtype=float)

    def moved(self, transform):
        return Box(self.name, transform @ self.transform, self.half)

    def vertices(self):
        return (CORNERS * self.half) @ self.transform[:3, :3].T + self.transform[:3, 3]

    def distances(self, points):
        local = (points - self.transform[:3, 3]) @ self.transform[:3, :3]
        return np.linalg.norm(np.maximum(np.abs(local) - self.half, 0.), axis=1)

    def contains(self, points, padding=0.):
        local = (points - self.transform[:3, 3]) @ self.transform[:3, :3]
        return np.all(np.abs(local) <= self.half + padding, axis=1)


def link_boxes(link, urdf):
    boxes = []
    for index, collision in enumerate(link.findall('collision')):
        geometry = collision.find('geometry')
        if geometry is None:
            continue
        transform = origin(collision.find('origin'))
        if geometry.find('box') is not None:
            half = np.array([float(v) for v in geometry.find('box').get('size').split()]) / 2
        elif geometry.find('cylinder') is not None:
            shape = geometry.find('cylinder')
            half = np.array([float(shape.get('radius'))]*2 + [float(shape.get('length'))/2])
        elif geometry.find('sphere') is not None:
            half = np.full(3, float(geometry.find('sphere').get('radius')))
        elif geometry.find('mesh') is not None:
            mesh = geometry.find('mesh')
            filename = mesh.get('filename')
            if filename.startswith('file://'):
                filename = filename[7:]
            elif filename.startswith('package://'):
                raise ValueError('Resolve package collision paths before loading: ' + filename)
            else:
                filename = str((urdf.parent / filename).resolve())
            low, high = mesh_bounds(filename)
            scale = np.array([float(v) for v in mesh.get('scale', '1 1 1').split()])
            a, b = low*scale, high*scale
            low, high = np.minimum(a, b), np.maximum(a, b)
            transform[:3, 3] += transform[:3, :3] @ ((low + high)/2)
            half = (high-low)/2
        else:
            raise ValueError('Unsupported URDF collision geometry')
        boxes.append(Box(link.get('name') + '/' + collision.get('name', str(index)), transform, half))
    return boxes


class RobotGeometry:
    def __init__(self, physical_urdf=None, calibrated_urdf=None):
        physical_urdf = Path(physical_urdf or REPO/'SW/robot/robot.physical.urdf')
        calibrated_urdf = Path(calibrated_urdf or REPO/'SW/simulation/robot_description/robocup.calibrated.urdf')
        physical, calibrated = ET.parse(physical_urdf).getroot(), ET.parse(calibrated_urdf).getroot()
        self.fixed = link_boxes(physical.find("link[@name='base_link']"), physical_urdf)
        self.tracer = list(self.fixed)
        # Self-filter only: the assembled rack encloses its wiring and open
        # interior. Keep this separate from physical collision boxes and Nav2.
        self.self_interiors = []
        joints = {j.find('child').get('link'): j for j in calibrated.findall('joint')}

        def fixed_transform(link):
            if link == 'base_link':
                return np.eye(4)
            joint = joints[link]
            if joint.get('type') not in ('fixed', 'continuous'):
                raise ValueError('Moving link cannot use a fixed collision pose: ' + link)
            return fixed_transform(joint.find('parent').get('link')) @ origin(joint.find('origin'))

        # Wheels and the individual rack boxes exist in the complete CAD model,
        # while the physical arm URDF supplies its actual joint convention.
        for link in calibrated.findall('link'):
            name = link.get('name')
            if (name.endswith('_wheel_link') or
                    name in ('left_wheel_link', 'right_wheel_link', 'rack_base_link',
                             'g2_link', 'mid360_link', 'camera_link')):
                local_boxes = link_boxes(link, calibrated_urdf)
                matrix = fixed_transform(name)
                boxes = [b.moved(matrix) for b in local_boxes]
                self.fixed.extend(boxes)
                if name == 'rack_base_link':
                    vertices = np.concatenate([b.vertices() for b in local_boxes])
                    low, high = vertices.min(axis=0), vertices.max(axis=0)
                    center = np.eye(4)
                    center[:3, 3] = (low+high)/2
                    self.self_interiors.append(Box('self_interior/rack_base_link',
                                                   matrix @ center, (high-low)/2))
                elif name.endswith('_wheel_link'):
                    self.tracer.extend(boxes)
        self.arm_links = {l.get('name'): link_boxes(l, physical_urdf) for l in physical.findall('link')
                          if l.get('name').startswith('piper_')}
        self.arm_joints = {j.find('child').get('link'): j for j in physical.findall('joint')}
        self.required_joints = {j.get('name') for j in physical.findall('joint') if j.get('type') == 'revolute'}
        vertices = np.concatenate([b.vertices() for b in self.tracer])
        self.tracer_low, self.tracer_high = vertices.min(axis=0), vertices.max(axis=0)

    def boxes(self, positions):
        transforms = {'base_link': np.eye(4)}
        def transform(link):
            if link in transforms:
                return transforms[link]
            joint = self.arm_joints[link]
            motion = np.eye(4)
            kind = joint.get('type')
            axis = np.array([float(v) for v in joint.find('axis').get('xyz', '1 0 0').split()]) if kind != 'fixed' else None
            if kind == 'revolute':
                if joint.get('name') not in positions:
                    raise ValueError('Missing measured arm joint: ' + joint.get('name'))
                angle = positions[joint.get('name')]
                axis /= np.linalg.norm(axis)
                x, y, z = axis
                skew = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
                motion[:3, :3] = np.eye(3) + np.sin(angle)*skew + (1-np.cos(angle))*(skew @ skew)
            elif kind == 'prismatic':
                limit = joint.find('limit')
                low, high = float(limit.get('lower')), float(limit.get('upper'))
                motion[:3, 3] = axis*((low+high)/2)
            transforms[link] = transform(joint.find('parent').get('link')) @ origin(joint.find('origin')) @ motion
            return transforms[link]
        out = list(self.fixed)
        for link, boxes in self.arm_links.items():
            matrix = transform(link)
            joint = self.arm_joints[link]
            for box in boxes:
                # No measured finger position: cover its complete allowed stroke.
                half = box.half.copy()
                if joint.get('type') == 'prismatic':
                    limit = joint.find('limit')
                    axis = np.array([float(v) for v in joint.find('axis').get('xyz').split()])
                    half += np.abs(box.transform[:3, :3].T @ axis) * (float(limit.get('upper'))-float(limit.get('lower')))/2
                out.append(Box(box.name, matrix @ box.transform, half))
        return out


def rectangle(boxes, margin=0.):
    vertices = np.concatenate([b.vertices() for b in boxes])
    low, high = vertices[:, :2].min(axis=0)-margin, vertices[:, :2].max(axis=0)+margin
    return [[float(high[0]), float(high[1])], [float(high[0]), float(low[1])],
            [float(low[0]), float(low[1])], [float(low[0]), float(high[1])]]


def convex_footprint(boxes):
    """Smallest convex XY outline containing every physical box vertex."""
    points = sorted(set(map(tuple, np.concatenate([b.vertices() for b in boxes])[:, :2])))
    if len(points) < 3:
        raise ValueError('Physical footprint requires at least three points')

    def half(sequence):
        out = []
        for p in sequence:
            while len(out) >= 2:
                a, b = out[-2:]
                cross = (b[0]-a[0])*(p[1]-a[1])-(b[1]-a[1])*(p[0]-a[0])
                if cross > 0.:
                    break
                out.pop()
            out.append(p)
        return out

    hull = half(points)[:-1]+half(reversed(points))[:-1]
    return [[float(x), float(y)] for x, y in hull]


def nearest(points, boxes):
    if not len(points):
        return float('inf'), None, None
    best, point, name = float('inf'), None, None
    for box in boxes:
        distances = box.distances(points)
        index = int(np.argmin(distances))
        if distances[index] < best:
            best, point, name = float(distances[index]), points[index].tolist(), box.name
    return best, point, name
