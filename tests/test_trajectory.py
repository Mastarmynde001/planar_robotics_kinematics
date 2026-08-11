import math
import numpy as np
import pytest
import kinematics_core_py as kine
from trajectory_planner import QuinticTrajectory1D, TrajectoryPlanner


def test_quintic_trajectory_1d_boundary_conditions():
    duration = 2.0
    quintic = QuinticTrajectory1D(duration)

    # At t = 0: s = 0, s_dot = 0, s_ddot = 0
    s_start, s_dot_start, s_ddot_start = quintic.evaluate(np.array([0.0]))
    assert math.isclose(s_start[0], 0.0, abs_tol=1e-7)
    assert math.isclose(s_dot_start[0], 0.0, abs_tol=1e-7)
    assert math.isclose(s_ddot_start[0], 0.0, abs_tol=1e-7)

    # At t = duration: s = 1, s_dot = 0, s_ddot = 0
    s_end, s_dot_end, s_ddot_end = quintic.evaluate(np.array([duration]))
    assert math.isclose(s_end[0], 1.0, abs_tol=1e-7)
    assert math.isclose(s_dot_end[0], 0.0, abs_tol=1e-7)
    assert math.isclose(s_ddot_end[0], 0.0, abs_tol=1e-7)


def test_line_trajectory_generation():
    arm = kine.Manipulator2D(1.0, 1.0)
    planner = TrajectoryPlanner(arm)

    start_pose = (1.5, 0.0)
    end_pose = (0.5, 1.0)
    traj = planner.plan_line_trajectory(start_pose, end_pose, duration=2.0, num_steps=50)

    assert len(traj["time"]) == 50
    assert math.isclose(traj["cartesian_x"][0], 1.5, abs_tol=1e-6)
    assert math.isclose(traj["cartesian_y"][0], 0.0, abs_tol=1e-6)
    assert math.isclose(traj["cartesian_x"][-1], 0.5, abs_tol=1e-6)
    assert math.isclose(traj["cartesian_y"][-1], 1.0, abs_tol=1e-6)

    # Check that initial and final velocity are approximately zero
    assert abs(traj["theta1_dot"][0]) < 0.1
    assert abs(traj["theta2_dot"][0]) < 0.1
    assert abs(traj["theta1_dot"][-1]) < 0.1
    assert abs(traj["theta2_dot"][-1]) < 0.1


def test_circle_trajectory_generation():
    arm = kine.Manipulator2D(1.0, 1.0)
    planner = TrajectoryPlanner(arm)

    center = (0.8, 0.3)
    radius = 0.4
    traj = planner.plan_circle_trajectory(center, radius, duration=3.0, num_steps=60)

    assert len(traj["time"]) == 60
    # Check that end-effector positions match FK positions
    for i in range(len(traj["time"])):
        fk_pose = arm.computeFK(traj["theta1"][i], traj["theta2"][i])
        assert math.isclose(fk_pose.x, traj["cartesian_x"][i], abs_tol=1e-5)
        assert math.isclose(fk_pose.y, traj["cartesian_y"][i], abs_tol=1e-5)


def test_out_of_bounds_trajectory_raises_exception():
    arm = kine.Manipulator2D(1.0, 1.0)
    planner = TrajectoryPlanner(arm)

    # Start pose is outside maximum reach of 2.0m
    with pytest.raises(ValueError) as exc_info:
        planner.plan_line_trajectory((1.0, 0.0), (3.0, 0.0), duration=2.0, num_steps=30)
    assert "out of reach" in str(exc_info.value).lower()
