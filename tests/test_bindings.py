#!/usr/bin/env python3
import math
import unittest
import kinematics_core_py as kine

class TestManipulator2DBindings(unittest.TestCase):

    def setUp(self):
        self.arm = kine.Manipulator2D(1.0, 1.0)

    def test_constructor_properties(self):
        self.assertAlmostEqual(self.arm.l1, 1.0)
        self.assertAlmostEqual(self.arm.l2, 1.0)

        with self.assertRaises(ValueError):
            kine.Manipulator2D(-1.0, 1.0)
        with self.assertRaises(ValueError):
            kine.Manipulator2D(1.0, 0.0)

    def test_compute_fk(self):
        # θ1=0, θ2=0 -> (2.0, 0.0)
        pose = self.arm.computeFK(0.0, 0.0)
        self.assertAlmostEqual(pose.x, 2.0, places=6)
        self.assertAlmostEqual(pose.y, 0.0, places=6)

        # Unpacking / indexing
        x, y = pose
        self.assertAlmostEqual(x, 2.0, places=6)
        self.assertAlmostEqual(y, 0.0, places=6)

        # θ1=0, θ2=π/2 -> (1.0, 1.0)
        pose2 = self.arm.computeFK(0.0, math.pi / 2.0)
        self.assertAlmostEqual(pose2.x, 1.0, places=6)
        self.assertAlmostEqual(pose2.y, 1.0, places=6)

    def test_compute_ik_known_positions(self):
        # Target (2.0, 0.0) -> (0.0, 0.0)
        js = self.arm.computeIK(2.0, 0.0)
        self.assertAlmostEqual(js.theta1, 0.0, places=6)
        self.assertAlmostEqual(js.theta2, 0.0, places=6)

        # Unpacking / indexing
        t1, t2 = js
        self.assertAlmostEqual(t1, 0.0, places=6)
        self.assertAlmostEqual(t2, 0.0, places=6)

    def test_compute_ik_elbow_configurations(self):
        target_x, target_y = 1.0, 1.0

        # Elbow Up (elbow_up=True) -> theta2 > 0
        js_up = self.arm.computeIK(target_x, target_y, elbow_up=True)
        self.assertGreater(js_up.theta2, 0.0)
        rec_pose_up = self.arm.computeFK(js_up)
        self.assertAlmostEqual(rec_pose_up.x, target_x, places=6)
        self.assertAlmostEqual(rec_pose_up.y, target_y, places=6)

        # Elbow Down (elbow_up=False) -> theta2 < 0
        js_down = self.arm.computeIK(target_x, target_y, elbow_up=False)
        self.assertLess(js_down.theta2, 0.0)
        rec_pose_down = self.arm.computeFK(js_down)
        self.assertAlmostEqual(rec_pose_down.x, target_x, places=6)
        self.assertAlmostEqual(rec_pose_down.y, target_y, places=6)

    def test_compute_ik_out_of_reach_exception(self):
        with self.assertRaises(ValueError) as ctx:
            self.arm.computeIK(3.0, 0.0)
        self.assertIn("out of reach", str(ctx.exception).lower())

    def test_compute_ik_singularity_exception(self):
        # Origin (0.0, 0.0) for equal link lengths (1.0, 1.0)
        with self.assertRaises(ValueError) as ctx:
            self.arm.computeIK(0.0, 0.0)
        self.assertIn("singularity", str(ctx.exception).lower())

    def test_jacobian_and_singularity_utilities(self):
        extended_state = kine.JointState(0.0, 0.0)
        det = self.arm.computeJacobianDeterminant(extended_state)
        self.assertAlmostEqual(det, 0.0, places=6)
        self.assertTrue(self.arm.isSingular(extended_state))

        bent_state = kine.JointState(0.0, math.pi / 2.0)
        det_bent = self.arm.computeJacobianDeterminant(bent_state)
        self.assertAlmostEqual(det_bent, 1.0, places=6)
        self.assertFalse(self.arm.isSingular(bent_state))

if __name__ == "__main__":
    unittest.main()
