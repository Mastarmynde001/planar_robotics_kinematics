import numpy as np
import kinematics_core_py as kine

class QuinticTrajectory1D:
    """Computes a smooth quintic polynomial trajectory profile s(t) in [0, 1]."""
    def __init__(self, duration: float):
        if duration <= 0:
            raise ValueError("Duration must be positive.")
        self.T = duration

    def evaluate(self, t: np.ndarray):
        """Returns tuple (s, s_dot, s_ddot) at time vector t."""
        tau = np.clip(t / self.T, 0.0, 1.0)
        s = 10.0 * tau**3 - 15.0 * tau**4 + 6.0 * tau**5
        s_dot = (30.0 * tau**2 - 60.0 * tau**3 + 30.0 * tau**4) / self.T
        s_ddot = (60.0 * tau - 180.0 * tau**2 + 120.0 * tau**3) / (self.T**2)
        return s, s_dot, s_ddot


class TrajectoryPlanner:
    """Generates Cartesian and Joint trajectories for Manipulator2D."""
    def __init__(self, arm: kine.Manipulator2D):
        self.arm = arm

    def plan_line_trajectory(self, start_pose: tuple, end_pose: tuple, duration: float = 2.0, num_steps: int = 100, elbow_up: bool = False):
        """Plans a straight line Cartesian trajectory between start_pose (x0, y0) and end_pose (x1, y1)."""
        t = np.linspace(0.0, duration, num_steps)
        quintic = QuinticTrajectory1D(duration)
        s, s_dot, s_ddot = quintic.evaluate(t)

        x0, y0 = start_pose
        x1, y1 = end_pose

        x_path = x0 + s * (x1 - x0)
        y_path = y0 + s * (y1 - y0)

        return self._solve_trajectory_ik(t, x_path, y_path, elbow_up)

    def plan_circle_trajectory(self, center: tuple, radius: float, start_angle: float = 0.0, end_angle: float = 2 * np.pi, duration: float = 3.0, num_steps: int = 150, elbow_up: bool = False):
        """Plans a circular arc Cartesian trajectory centered at (cx, cy) with radius R."""
        t = np.linspace(0.0, duration, num_steps)
        quintic = QuinticTrajectory1D(duration)
        s, _, _ = quintic.evaluate(t)

        cx, cy = center
        phi = start_angle + s * (end_angle - start_angle)

        x_path = cx + radius * np.cos(phi)
        y_path = cy + radius * np.sin(phi)

        return self._solve_trajectory_ik(t, x_path, y_path, elbow_up)

    def _solve_trajectory_ik(self, t: np.ndarray, x_path: np.ndarray, y_path: np.ndarray, elbow_up: bool):
        num_steps = len(t)
        theta1 = np.zeros(num_steps)
        theta2 = np.zeros(num_steps)

        elbow_x = np.zeros(num_steps)
        elbow_y = np.zeros(num_steps)
        ee_x = np.zeros(num_steps)
        ee_y = np.zeros(num_steps)

        l1 = self.arm.l1

        for i in range(num_steps):
            x_target = x_path[i]
            y_target = y_path[i]

            try:
                js = self.arm.computeIK(x_target, y_target, elbow_up=elbow_up)
            except ValueError as e:
                raise ValueError(f"Trajectory planning failed at step {i} (t={t[i]:.2f}s, x={x_target:.2f}, y={y_target:.2f}): {e}")

            theta1[i] = js.theta1
            theta2[i] = js.theta2

            # Compute link joint positions for visualization
            elbow_x[i] = l1 * np.cos(js.theta1)
            elbow_y[i] = l1 * np.sin(js.theta1)

            fk_pose = self.arm.computeFK(js)
            ee_x[i] = fk_pose.x
            ee_y[i] = fk_pose.y

        # Compute numerical derivatives for velocities and accelerations
        dt = t[1] - t[0] if num_steps > 1 else 1.0
        theta1_dot = np.gradient(theta1, dt)
        theta2_dot = np.gradient(theta2, dt)

        theta1_ddot = np.gradient(theta1_dot, dt)
        theta2_ddot = np.gradient(theta2_dot, dt)

        return {
            "time": t,
            "cartesian_x": x_path,
            "cartesian_y": y_path,
            "theta1": theta1,
            "theta2": theta2,
            "theta1_dot": theta1_dot,
            "theta2_dot": theta2_dot,
            "theta1_ddot": theta1_ddot,
            "theta2_ddot": theta2_ddot,
            "elbow_x": elbow_x,
            "elbow_y": elbow_y,
            "ee_x": ee_x,
            "ee_y": ee_y,
        }


if __name__ == "__main__":
    arm = kine.Manipulator2D(1.0, 1.0)
    planner = TrajectoryPlanner(arm)

    print("Testing Line Trajectory Planning...")
    traj = planner.plan_line_trajectory((1.5, 0.0), (0.5, 1.0), duration=2.0, num_steps=50)
    print(f"Successfully generated line trajectory with {len(traj['time'])} steps.")

    print("Testing Circle Trajectory Planning...")
    traj_circle = planner.plan_circle_trajectory((0.8, 0.0), 0.5, duration=3.0, num_steps=50)
    print(f"Successfully generated circle trajectory with {len(traj_circle['time'])} steps.")
