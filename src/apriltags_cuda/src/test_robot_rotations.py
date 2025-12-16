import unittest
import robot_rotations as rotations
import numpy as np


class TestRotations(unittest.TestCase):

    def test_leftfrontnominal(self):
        """
        Tests the nominal coord transformation from cam coords to robot coords
        for the left front camera.
        """

        LF_roll = -90
        LF_pitch = 90
        LF_yaw = 0
        R = rotations.compose_rotations_xyz(LF_roll, LF_pitch, LF_yaw)

        vc = np.array([0, 0, 1])
        robot_coords = R @ vc
        expected = np.array([1.0, 0.0, 0.0])
        self.assertTrue(
            np.allclose(robot_coords, expected),
            msg=f"Got {robot_coords}, expected {expected}",
        )

        vc = np.array([0, 1, 0])
        robot_coords = R @ vc
        expected = np.array([0.0, 0.0, -1.0])
        self.assertTrue(
            np.allclose(robot_coords, expected),
            msg=f"Got {robot_coords}, expected {expected}",
        )

        vc = np.array([1, 0, 0])
        robot_coords = R @ vc
        expected = np.array([0.0, -1.0, 0.0])
        self.assertTrue(
            np.allclose(robot_coords, expected),
            msg=f"Got {robot_coords}, expected {expected}",
        )

    def test_rightfrontnominal(self):
        """
        Tests the nominal coord transformation from cam coords to robot coords
        for the right front camera.
        """

        LF_roll = -90
        LF_pitch = 90
        LF_yaw = 0
        R = rotations.compose_rotations_xyz(LF_roll, LF_pitch, LF_yaw)

        vc = np.array([0, 0, 1])
        robot_coords = R @ vc
        expected = np.array([1.0, 0.0, 0.0])
        self.assertTrue(
            np.allclose(robot_coords, expected),
            msg=f"Got {robot_coords}, expected {expected}",
        )

        vc = np.array([0, 1, 0])
        robot_coords = R @ vc
        expected = np.array([0.0, 0.0, -1.0])
        self.assertTrue(
            np.allclose(robot_coords, expected),
            msg=f"Got {robot_coords}, expected {expected}",
        )

        vc = np.array([1, 0, 0])
        robot_coords = R @ vc
        expected = np.array([0.0, -1.0, 0.0])
        self.assertTrue(
            np.allclose(robot_coords, expected),
            msg=f"Got {robot_coords}, expected {expected}",
        )

    def test_leftbacknominal(self):
        """
        Tests the nominal coord transformation from cam coords to robot coords
        for the left back camera.
        """

        LF_roll = -90
        LF_pitch = -90
        LF_yaw = 0
        R = rotations.compose_rotations_xyz(LF_roll, LF_pitch, LF_yaw)

        vc = np.array([0, 0, 1])
        robot_coords = R @ vc
        expected = np.array([-1.0, 0.0, 0.0])
        self.assertTrue(
            np.allclose(robot_coords, expected),
            msg=f"Got {robot_coords}, expected {expected}",
        )

        vc = np.array([0, 1, 0])
        robot_coords = R @ vc
        expected = np.array([0.0, 0.0, -1.0])
        self.assertTrue(
            np.allclose(robot_coords, expected),
            msg=f"Got {robot_coords}, expected {expected}",
        )

        vc = np.array([1, 0, 0])
        robot_coords = R @ vc
        expected = np.array([0.0, 1.0, 0.0])
        self.assertTrue(
            np.allclose(robot_coords, expected),
            msg=f"Got {robot_coords}, expected {expected}",
        )

    def test_rightbacknominal(self):
        """
        Tests the nominal coord transformation from cam coords to robot coords
        for the right back camera.
        """

        LF_roll = -90
        LF_pitch = -90
        LF_yaw = 0
        R = rotations.compose_rotations_xyz(LF_roll, LF_pitch, LF_yaw)

        vc = np.array([0, 0, 1])
        robot_coords = R @ vc
        expected = np.array([-1.0, 0.0, 0.0])
        self.assertTrue(
            np.allclose(robot_coords, expected),
            msg=f"Got {robot_coords}, expected {expected}",
        )

        vc = np.array([0, 1, 0])
        robot_coords = R @ vc
        expected = np.array([0.0, 0.0, -1.0])
        self.assertTrue(
            np.allclose(robot_coords, expected),
            msg=f"Got {robot_coords}, expected {expected}",
        )

        vc = np.array([1, 0, 0])
        robot_coords = R @ vc
        expected = np.array([0.0, 1.0, 0.0])
        self.assertTrue(
            np.allclose(robot_coords, expected),
            msg=f"Got {robot_coords}, expected {expected}",
        )


if __name__ == "__main__":
    unittest.main()
