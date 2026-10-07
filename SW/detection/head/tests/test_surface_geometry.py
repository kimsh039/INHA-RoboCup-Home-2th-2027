import math
import unittest
import numpy as np
from robocup_head_detection.surface_geometry import SurfaceConfig, find_surfaces, min_area_rect


def ring_cloud(tables, sensor=(0.0, 0.0, 1.34), rings=np.radians(np.arange(-52, 8, 1.0)),
               azimuths=np.radians(np.arange(-180, 180, 1.06)), walls=3.0, floor=0.0):
    """Ring lidar hitting boxes' tops (cx, cy, yaw, length, width, height), a square room and the floor."""
    sx, sy, sz = sensor
    pts = []
    for el in rings:
        for az in azimuths:
            d = np.array([math.cos(el)*math.cos(az), math.cos(el)*math.sin(az), math.sin(el)])
            best = None
            if d[2] < 0:                                   # floor
                best = (floor-sz)/d[2]
            for cx, cy, yaw, length, width, height in tables:
                if d[2] >= 0:
                    continue
                t = (height-sz)/d[2]
                x, y = sx+t*d[0]-cx, sy+t*d[1]-cy
                u, v = x*math.cos(yaw)+y*math.sin(yaw), -x*math.sin(yaw)+y*math.cos(yaw)
                if abs(u) <= length/2 and abs(v) <= width/2 and (best is None or t < best):
                    best = t
            for axis in (0, 1):                            # walls at +-walls
                if abs(d[axis]) > 1e-9:
                    t = (math.copysign(walls, d[axis])-(sx, sy)[axis])/d[axis]
                    if t > 0 and (best is None or t < best):
                        best = t
            if best is not None:
                pts.append((sx+best*d[0], sy+best*d[1], sz+best*d[2]))
    return np.array(pts)


class SurfaceGeometryTests(unittest.TestCase):
    def test_min_area_rect_rotated(self):
        a = math.radians(30)
        local = np.array([[x, y] for x in np.linspace(-.8, .8, 17) for y in np.linspace(-.4, .4, 9)])
        xy = local @ np.array([[math.cos(a), math.sin(a)], [-math.sin(a), math.cos(a)]]) + (1, 2)
        centre, yaw, length, width, corners = min_area_rect(xy)
        np.testing.assert_allclose(centre, (1, 2), atol=1e-9)
        self.assertAlmostEqual(yaw, a, places=9)
        self.assertAlmostEqual(length, 1.6, places=9)
        self.assertAlmostEqual(width, 0.8, places=9)
        # Counter-clockwise corners: positive signed area.
        x, y = corners[:, 0], corners[:, 1]
        self.assertGreater(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1)), 0)

    def test_two_tables_and_walls(self):
        tables = [(1.8, -1.0, 0.0, 1.6, 0.8, 0.72), (1.6, 1.6, math.radians(30), 1.2, 0.6, 0.76)]
        cloud = ring_cloud(tables)
        found = find_surfaces(cloud, SurfaceConfig(min_layer_points=40))
        self.assertEqual(len(found), 2)                 # wall rings rejected
        by_y = sorted(found, key=lambda s: s.centre[1])
        for surface, (cx, cy, yaw, length, width, height) in zip(by_y, sorted(tables, key=lambda t: t[1])):
            self.assertAlmostEqual(surface.height, height, delta=0.005)
            self.assertLess(math.hypot(surface.centre[0]-cx, surface.centre[1]-cy), 0.15)
            self.assertLess(abs(math.atan2(math.sin(2*(surface.yaw-yaw)), math.cos(2*(surface.yaw-yaw))))/2,
                            math.radians(6))
            self.assertLess(surface.length, length+0.03)
            self.assertGreater(surface.width, width*0.6)

    def test_floor_offset_and_near_edge_observed(self):
        floor = -0.1425
        cloud = ring_cloud([(1.8, -1.0, 0.0, 1.6, 0.8, 0.72+floor)], sensor=(0, 0, 1.34+floor), floor=floor)
        (surface,) = find_surfaces(cloud, SurfaceConfig(floor_z=floor, min_layer_points=40))
        self.assertAlmostEqual(surface.height, 0.72, delta=0.005)
        self.assertAlmostEqual(surface.z, 0.72+floor, delta=0.005)
        # The long side facing the sensor (y = -0.6) is seen along most of its length.
        near = max(range(4), key=lambda i: surface.edge(i)[2][1])
        _, _, mid, normal = surface.edge(near)
        self.assertGreater(normal[1], 0.9)
        self.assertGreater(surface.edge_observed[near], 0.6)

    def test_scan_end_is_not_a_boundary(self):
        # Returns stop at x = 2.0 (range limit / occlusion) although the table reaches 2.6.
        cloud = ring_cloud([(1.8, -1.0, 0.0, 1.6, 0.8, 0.72)])
        (surface,) = find_surfaces(cloud[cloud[:, 0] < 2.0], SurfaceConfig(min_layer_points=40))
        cut = max(range(4), key=lambda i: surface.edge(i)[2][0])
        self.assertAlmostEqual(surface.edge(cut)[2][0], 2.0, delta=0.03)
        self.assertLess(surface.edge_observed[cut], 0.1)
        start = min(range(4), key=lambda i: surface.edge(i)[2][0])   # x = 1.0, the real end
        self.assertGreater(surface.edge_observed[start], 0.8)

    def test_empty_and_nan(self):
        self.assertEqual(find_surfaces(np.full((10, 3), np.nan)), [])
        self.assertEqual(find_surfaces(np.zeros((0, 3))), [])


if __name__ == '__main__':
    unittest.main()
