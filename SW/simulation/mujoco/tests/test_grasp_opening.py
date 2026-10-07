import unittest
import numpy as np
from manipulation.grasp_opening import cube_opening_check


class OpeningTest(unittest.TestCase):
    def check(self, translation, size=.04, rotation=None):
        grasp = {'translation': translation, 'rotation_matrix': np.eye(3) if rotation is None else rotation}
        return cube_opening_check(grasp, [size]*3, .07)['fits']

    def test_centered_cube_and_oversize(self):
        self.assertTrue(self.check([0, 0, 0]))
        self.assertFalse(self.check([0, 0, 0], size=.08))

    def test_offcenter_cube(self):
        # 폭만 비교하면 놓치는 편심에 의한 한쪽 돌출을 검사한다.
        self.assertFalse(self.check([0, .02, 0]))
        self.assertTrue(self.check([0, .01, 0]))

    def test_diagonal_cube(self):
        angle = np.pi / 4
        rotation = np.array([[np.cos(angle), -np.sin(angle), 0],
                             [np.sin(angle), np.cos(angle), 0], [0, 0, 1]])
        self.assertTrue(self.check([0, 0, 0], rotation=rotation))
        self.assertFalse(self.check([0, 0, 0], size=.06, rotation=rotation))
