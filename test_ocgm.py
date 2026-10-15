"""Focused numerical and export checks, independent of Gazebo."""
import math
import unittest

import numpy as np

from myworld_ocgm_660610823 import OccupancyGrid


class MappingTests(unittest.TestCase):
    def test_hit_free_unknown_and_negative_coordinates(self):
        grid = OccupancyGrid([-2, 3, -2, 3], 1)
        grid.ray(-1.5, -1.5, 0, 2, True)
        p = grid.probabilities()
        np.testing.assert_allclose(p[0, :3], [0.3, 0.3, 0.7])
        self.assertEqual(p[4, 4], 0.5)

    def test_no_return_and_clipped_hit_do_not_occupy_boundary(self):
        for hit in (False, True):
            grid = OccupancyGrid([0, 3, 0, 3], 1)
            grid.ray(0.5, 0.5, 0, 100, hit)
            np.testing.assert_allclose(grid.probabilities()[0], [0.3, 0.3, 0.3])
            self.assertFalse((grid.probabilities() > 0.5).any())

    def test_rotated_sensor_mount(self):
        grid = OccupancyGrid([-2, 4, -2, 4], 1)
        metadata = dict(sensor_x=1, sensor_y=0, sensor_yaw=-math.pi / 2,
                        range_min=0.1, range_max=10)
        grid.scan([0.5, 0.5, math.pi / 2, 2, 0], metadata)
        # Robot at (0.5,0.5), sensor at (0.5,1.5), beam points east to (2.5,1.5).
        np.testing.assert_allclose(grid.probabilities()[3, 2:5], [0.3, 0.3, 0.7])
        self.assertEqual(grid.probabilities()[2, 2], 0.5)

    def test_invalid_and_infinite_ranges(self):
        grid = OccupancyGrid([0, 3, 0, 3], 1)
        metadata = dict(sensor_x=0, sensor_y=0, sensor_yaw=0, range_min=0.15, range_max=2)
        grid.scan([0.5, 0.5, 0, float('nan'), 0, -math.inf, 0, 0.1, 0], metadata)
        self.assertTrue((grid.probabilities() == 0.5).all())
        grid.scan([0.5, 0.5, 0, math.inf, 0], metadata)
        np.testing.assert_allclose(grid.probabilities()[0], [0.3, 0.3, 0.3])

    def test_probability_stays_finite_and_origin_is_checked(self):
        grid = OccupancyGrid([0, 3, 0, 3], 1)
        for _ in range(100):
            grid.ray(0.5, 0.5, 0, 2, True)
        p = grid.probabilities()
        self.assertTrue(np.isfinite(p).all())
        self.assertTrue(((p >= 0) & (p <= 1)).all())
        with self.assertRaises(ValueError):
            grid.ray(-0.1, 0.5, 0, 2, True)


if __name__ == '__main__':
    unittest.main()
