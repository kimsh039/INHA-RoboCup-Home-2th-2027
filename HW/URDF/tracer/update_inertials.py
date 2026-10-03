#!/usr/bin/env python3
"""Apply documented 30 kg Tracer approximation; no third-party dependencies."""
from pathlib import Path
import json
import math
import xml.etree.ElementTree as E

HERE = Path(__file__).resolve().parent
NS = {'c': 'http://www.collada.org/2005/11/COLLADASchema'}


def rotation(rpy):
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return [[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
            [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
            [-sp, cp*sr, cp*cr]]


def mesh_bounds(path, rpy, xyz):
    root = E.parse(path).getroot()
    # These checked-in meshes store their geometry directly in mesh coordinates.
    for node in root.findall('.//c:visual_scene//c:node', NS):
        for tag in ('matrix', 'translate', 'rotate', 'scale'):
            if node.find('c:'+tag, NS) is not None:
                raise ValueError('Scene transforms require explicit handling: '+str(path))
    unit = float(root.find('c:asset/c:unit', NS).get('meter'))
    points = []
    R = rotation(rpy)
    for geometry in root.findall('c:library_geometries/c:geometry', NS):
        mesh = geometry.find('c:mesh', NS)
        if mesh is None:
            continue
        for vertices in mesh.findall('c:vertices', NS):
            position = vertices.find("c:input[@semantic='POSITION']", NS)
            source = mesh.find("c:source[@id='%s']" % position.get('source').lstrip('#'), NS)
            values = [float(v) for v in source.find('c:float_array', NS).text.split()]
            accessor = source.find('c:technique_common/c:accessor', NS)
            stride, offset = int(accessor.get('stride', '3')), int(accessor.get('offset', '0'))
            for i in range(int(accessor.get('count'))):
                v = [unit*values[offset+i*stride+j] for j in range(3)]
                points.append([sum(R[a][b]*v[b] for b in range(3))+xyz[a] for a in range(3)])
    if not points:
        raise ValueError('No mesh positions: '+str(path))
    low = [min(v[a] for v in points) for a in range(3)]
    high = [max(v[a] for v in points) for a in range(3)]
    return [(low[a]+high[a])/2 for a in range(3)], [high[a]-low[a] for a in range(3)]


def main():
    tree = E.parse(HERE/'tracer_v1.urdf')
    root = tree.getroot()
    report = {'source': 'https://docs.trossenrobotics.com/agilex_tracer_docs/specifications.html',
              'nominal_total_mass_kg': 30.0,
              'assumption': 'body 26 kg; two drive wheels 1.5 kg each; four forks and four caster wheels 0.125 kg each. Not measured component masses.',
              'links': {}}
    for link in root.findall('link'):
        name = link.get('name')
        inertial = link.find('inertial')
        if inertial is None:
            continue
        visual = root.find("link[@name='base_link']/visual") if name == 'inertial_link' else link.find('visual')
        mesh = visual.find('geometry/mesh')
        origin = visual.find('origin')
        rpy = [float(v) for v in (origin.get('rpy', '0 0 0') if origin is not None else '0 0 0').split()]
        xyz = [float(v) for v in (origin.get('xyz', '0 0 0') if origin is not None else '0 0 0').split()]
        center, size = mesh_bounds(HERE/mesh.get('filename'), rpy, xyz)
        if name == 'inertial_link':
            mass, kind = 26.0, 'uniform bounding box'
            # Original Tracer's mesh envelope also contains its top mounting rails.
            inertia = [mass*(size[(a+1)%3]**2+size[(a+2)%3]**2)/12 for a in range(3)]
        elif name in ('left_wheel_link', 'right_wheel_link'):
            mass, kind, axis = 1.5, 'solid cylinder, local Y axis', 1
            radius = max(size[0], size[2])/2
            inertia = [mass*(3*radius**2+size[axis]**2)/12]*3
            inertia[axis] = mass*radius**2/2
        elif name.endswith('_wheel_link'):
            mass, kind, axis = 0.125, 'solid cylinder, local Z axis', 2
            radius = max(size[0], size[1])/2
            inertia = [mass*(3*radius**2+size[axis]**2)/12]*3
            inertia[axis] = mass*radius**2/2
        else:
            mass, kind = 0.125, 'uniform bounding box'
            inertia = [mass*(size[(a+1)%3]**2+size[(a+2)%3]**2)/12 for a in range(3)]
        assert min(size)>0 and min(inertia)>0
        assert max(inertia) <= sum(inertia)-max(inertia)+1e-12
        inertial.find('mass').set('value', format(mass,'.12g'))
        o = inertial.find('origin')
        if o is None:
            o = E.SubElement(inertial, 'origin')
        o.set('xyz', ' '.join(format(v,'.12g') for v in center)); o.set('rpy', '0 0 0')
        tensor = inertial.find('inertia')
        for attr, value in zip(('ixx','iyy','izz'), inertia):
            tensor.set(attr, format(value,'.12g'))
        for attr in ('ixy','ixz','iyz'):
            tensor.set(attr, '0')
        report['links'][name] = {'mass_kg': mass, 'approximation': kind,
                                 'center_m': center, 'mesh_envelope_m': size,
                                 'diagonal_inertia_kg_m2': inertia}
    total = sum(v['mass_kg'] for v in report['links'].values())
    assert math.isclose(total,30.0,abs_tol=1e-10)
    report['total_mass_kg'] = total
    E.indent(tree)
    tree.write(HERE/'tracer_v1.urdf', encoding='utf-8', xml_declaration=True)
    (HERE/'inertial_report.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    print('Tracer total mass:', total, 'kg')


if __name__ == '__main__':
    main()
