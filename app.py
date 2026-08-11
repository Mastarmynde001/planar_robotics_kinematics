#!/usr/bin/env python3
import os
import sys
import numpy as np
import matplotlib
# Use Agg backend if headless display
if os.environ.get('DISPLAY', '') == '':
    matplotlib.use('Agg')

import matplotlib.pyplot as plt
import matplotlib.animation as animation
import kinematics_core_py as kine
from trajectory_planner import TrajectoryPlanner


def build_simulation_app():
    # Instantiate 2-DOF Planar Robot (l1 = 1.0m, l2 = 1.0m)
    arm = kine.Manipulator2D(1.0, 1.0)
    planner = TrajectoryPlanner(arm)

    # Generate Cartesian Trajectory: Circular arc trajectory
    # Center (0.8, 0.5), Radius 0.5m
    print("[Simulation] Generating quintic polynomial trajectory...")
    traj = planner.plan_circle_trajectory(
        center=(0.8, 0.3),
        radius=0.5,
        start_angle=0.0,
        end_angle=1.5 * np.pi,
        duration=3.0,
        num_steps=120,
        elbow_up=False
    )

    t = traj["time"]
    theta1 = traj["theta1"]
    theta2 = traj["theta2"]
    theta1_dot = traj["theta1_dot"]
    theta2_dot = traj["theta2_dot"]
    theta1_ddot = traj["theta1_ddot"]
    theta2_ddot = traj["theta2_ddot"]

    elbow_x = traj["elbow_x"]
    elbow_y = traj["elbow_y"]
    ee_x = traj["ee_x"]
    ee_y = traj["ee_y"]

    # Set up Matplotlib Figure with 4 Subplots (Main 2D View + 3 Kinematics Graphs)
    fig = plt.figure(figsize=(14, 8), dpi=100)
    fig.suptitle("2-DOF Planar Robot Manipulator - Trajectory Execution & Kinematics", fontsize=14, fontweight='bold')

    gs = fig.add_gridspec(3, 2, width_ratios=[1.3, 1.0])

    ax_arm = fig.add_subplot(gs[:, 0])
    ax_pos = fig.add_subplot(gs[0, 1])
    ax_vel = fig.add_subplot(gs[1, 1])
    ax_acc = fig.add_subplot(gs[2, 1])

    # --- 1. Main 2D Arm Plot Configuration ---
    ax_arm.set_title("Cartesian Arm Movement & Workspace Trail", fontsize=11, fontweight='bold')
    ax_arm.set_xlabel("X Position (m)", fontsize=10)
    ax_arm.set_ylabel("Y Position (m)", fontsize=10)
    ax_arm.set_xlim(-0.5, 2.2)
    ax_arm.set_ylim(-0.8, 1.8)
    ax_arm.set_aspect('equal')
    ax_arm.grid(True, linestyle='--', alpha=0.6)

    # Workspace boundaries
    R_max = arm.l1 + arm.l2
    R_min = abs(arm.l1 - arm.l2)
    outer_circle = plt.Circle((0, 0), R_max, color='gray', linestyle=':', fill=False, linewidth=1.5, label=f"Max Reach ({R_max}m)")
    ax_arm.add_patch(outer_circle)

    # Target path
    ax_arm.plot(traj["cartesian_x"], traj["cartesian_y"], 'r--', linewidth=1.5, alpha=0.7, label="Planned Target Path")

    # Robot links and joints elements
    base_marker, = ax_arm.plot(0, 0, 'ks', markersize=10, label="Base Joint")
    link1_line, = ax_arm.plot([], [], 'o-', color='#1f77b4', linewidth=4, markersize=8, label="Link 1 (l1)")
    link2_line, = ax_arm.plot([], [], 'o-', color='#ff7f0e', linewidth=4, markersize=8, label="Link 2 (l2)")
    trail_line, = ax_arm.plot([], [], 'g-', linewidth=2, alpha=0.8, label="EE Trail")
    ee_marker, = ax_arm.plot([], [], 'ro', markersize=7)

    ax_arm.legend(loc='upper left', fontsize=8)

    # --- 2. Kinematics Profile Plots ---
    # Joint Positions
    ax_pos.set_title("Joint Positions (rad)", fontsize=10, fontweight='bold')
    ax_pos.plot(t, theta1, 'b-', label=r"$\theta_1$")
    ax_pos.plot(t, theta2, 'orange', label=r"$\theta_2$")
    pos_cursor = ax_pos.axvline(t[0], color='red', linestyle='--', alpha=0.7)
    ax_pos.set_ylabel("Angle (rad)", fontsize=9)
    ax_pos.grid(True, linestyle='--', alpha=0.5)
    ax_pos.legend(loc='upper right', fontsize=8)

    # Joint Velocities
    ax_vel.set_title("Joint Velocities (rad/s)", fontsize=10, fontweight='bold')
    ax_vel.plot(t, theta1_dot, 'b-', label=r"$\dot{\theta}_1$")
    ax_vel.plot(t, theta2_dot, 'orange', label=r"$\dot{\theta}_2$")
    vel_cursor = ax_vel.axvline(t[0], color='red', linestyle='--', alpha=0.7)
    ax_vel.set_ylabel("Vel (rad/s)", fontsize=9)
    ax_vel.grid(True, linestyle='--', alpha=0.5)
    ax_vel.legend(loc='upper right', fontsize=8)

    # Joint Accelerations
    ax_acc.set_title("Joint Accelerations (rad/s²)", fontsize=10, fontweight='bold')
    ax_acc.plot(t, theta1_ddot, 'b-', label=r"$\ddot{\theta}_1$")
    ax_acc.plot(t, theta2_ddot, 'orange', label=r"$\ddot{\theta}_2$")
    acc_cursor = ax_acc.axvline(t[0], color='red', linestyle='--', alpha=0.7)
    ax_acc.set_xlabel("Time (s)", fontsize=9)
    ax_acc.set_ylabel("Acc (rad/s²)", fontsize=9)
    ax_acc.grid(True, linestyle='--', alpha=0.5)
    ax_acc.legend(loc='upper right', fontsize=8)

    plt.tight_layout()

    # --- 3. Animation Update Function ---
    def update(frame):
        # Update Arm links
        ex, ey = elbow_x[frame], elbow_y[frame]
        eex, eey = ee_x[frame], ee_y[frame]

        link1_line.set_data([0, ex], [0, ey])
        link2_line.set_data([ex, eex], [ey, eey])

        # Update Trail
        trail_line.set_data(ee_x[:frame+1], ee_y[:frame+1])
        ee_marker.set_data([eex], [eey])

        # Update Time Cursor lines
        time_curr = t[frame]
        pos_cursor.set_xdata([time_curr, time_curr])
        vel_cursor.set_xdata([time_curr, time_curr])
        acc_cursor.set_xdata([time_curr, time_curr])

        return link1_line, link2_line, trail_line, ee_marker, pos_cursor, vel_cursor, acc_cursor

    # Initialize frame
    update(0)

    # Save visual snapshot
    snapshot_path = "trajectory_snapshot.png"
    plt.savefig(snapshot_path, dpi=200, bbox_inches='tight')
    print(f"[Simulation] Saved snapshot image to {snapshot_path}")

    # Generate and save animation GIF
    print("[Simulation] Rendering animated GIF...")
    anim = animation.FuncAnimation(fig, update, frames=len(t), interval=30, blit=True)
    anim_path = "trajectory_animation.gif"
    anim.save(anim_path, writer='pillow', fps=30)
    print(f"[Simulation] Saved animation GIF to {anim_path}")

    if os.environ.get('DISPLAY', '') != '':
        plt.show()

    return snapshot_path, anim_path


if __name__ == "__main__":
    build_simulation_app()
