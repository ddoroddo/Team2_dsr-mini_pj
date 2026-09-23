#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
=====================================================================
 Doosan DSR Service & Gripper Simulator Bridge for Gazebo Simulation
=====================================================================
Provides full Doosan ROS 2 services & Gripper services so Team2 scripts
(gripper_repeat.py, pick_place_hardcoded.py, motion_seq.py, vision_pick_place.py)
can run seamlessly in the Gazebo simulation environment without physical robot hardware.

Services Provided:
  - /{namespace}/dsr_controller2/system/set_robot_mode (SetRobotMode)
  - /{namespace}/dsr_controller2/system/get_robot_mode (GetRobotMode)
  - /{namespace}/dsr_controller2/system/get_robot_state (GetRobotState)
  - /{namespace}/dsr_controller2/aux_control/get_current_posj (GetCurrentPosj)
  - /{namespace}/dsr_controller2/aux_control/get_current_posx (GetCurrentPosx)
  - /{namespace}/dsr_controller2/motion/move_joint (MoveJoint)
  - /{namespace}/dsr_controller2/motion/move_line (MoveLine)
  - /{namespace}/dsr_controller2/motion/move_jointx (MoveJointx)
  - /{namespace}/dsr_controller2/motion/check_motion (CheckMotion)
  - /{namespace}/gripper/cmd (GripperCmd)

Controller Topics Used:
  - /dsr_position_controller/commands (std_msgs/Float64MultiArray)
  - /gripper_controller/commands (std_msgs/Float64MultiArray)
  - /joint_states (sensor_msgs/JointState)
=====================================================================
"""

import math
import time
import threading
import numpy as np
from scipy.spatial.transform import Rotation as R

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from dsr_msgs2.srv import (
    SetRobotMode, GetRobotMode, GetRobotState,
    GetCurrentPosj, GetCurrentPosx,
    MoveJoint, MoveLine, MoveJointx, CheckMotion
)
from dsr_gripper_interfaces.srv import GripperCmd


class DsrSimBridge(Node):
    def __init__(self):
        super().__init__('dsr_sim_bridge')

        self.declare_parameter('namespace', 'dsr01')
        ns = self.get_parameter('namespace').get_parameter_value().string_value
        self.ns = ns if ns else 'dsr01'

        # Current joints in radians [j1, j2, j3, j4, j5, j6]
        self.current_joints = [0.0, 0.0, 1.5708, 0.0, 1.5708, 0.0]
        self.joint_state_received = False
        self.is_moving = False
        self.lock = threading.Lock()

        # Subscribe to Gazebo joint states
        self.sub_joint = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_cb,
            10
        )

        # Publisher to Gazebo robot position controller
        self.pub_arm = self.create_publisher(
            Float64MultiArray,
            '/dsr_position_controller/commands',
            10
        )

        # Publisher to Gazebo gripper controller (4 joints: [rh_r1, rh_l1, rh_r2, rh_l2])
        self.pub_gripper = self.create_publisher(
            Float64MultiArray,
            '/gripper_controller/commands',
            10
        )

        # Setup Doosan System & Aux Services
        prefix = f"/{self.ns}/dsr_controller2"
        self.srv_set_mode = self.create_service(
            SetRobotMode, f"{prefix}/system/set_robot_mode", self.cb_set_robot_mode
        )
        self.srv_get_mode = self.create_service(
            GetRobotMode, f"{prefix}/system/get_robot_mode", self.cb_get_robot_mode
        )
        self.srv_get_state = self.create_service(
            GetRobotState, f"{prefix}/system/get_robot_state", self.cb_get_robot_state
        )
        self.srv_get_posj = self.create_service(
            GetCurrentPosj, f"{prefix}/aux_control/get_current_posj", self.cb_get_current_posj
        )
        self.srv_get_posx = self.create_service(
            GetCurrentPosx, f"{prefix}/aux_control/get_current_posx", self.cb_get_current_posx
        )

        # Setup Doosan Motion Services
        self.srv_move_joint = self.create_service(
            MoveJoint, f"{prefix}/motion/move_joint", self.cb_move_joint
        )
        self.srv_move_line = self.create_service(
            MoveLine, f"{prefix}/motion/move_line", self.cb_move_line
        )
        self.srv_move_jointx = self.create_service(
            MoveJointx, f"{prefix}/motion/move_jointx", self.cb_move_jointx
        )
        self.srv_check_motion = self.create_service(
            CheckMotion, f"{prefix}/motion/check_motion", self.cb_check_motion
        )

        # Setup Gripper Service
        self.srv_gripper = self.create_service(
            GripperCmd, f"/{self.ns}/gripper/cmd", self.cb_gripper_cmd
        )

        # Robot Forward Kinematics Transformation Matrices
        self._init_fk_transforms()

        self.get_logger().info("=" * 65)
        self.get_logger().info(f" [DSR Sim Bridge] Ready for namespace '{self.ns}'")
        self.get_logger().info(f"   Services mapped to /{self.ns}/dsr_controller2/* & /{self.ns}/gripper/cmd")
        self.get_logger().info("   Translates DSR commands directly to Gazebo ros2_control")
        self.get_logger().info("=" * 65)

    def _init_fk_transforms(self):
        """Precompute FK relative transforms based on official e0509 URDF."""
        def tf(xyz, rpy):
            mat = np.eye(4)
            mat[:3, :3] = R.from_euler('xyz', rpy).as_matrix()
            mat[:3, 3] = xyz
            return mat

        self.T_base_1 = tf([0, 0, 0.2045], [0, 0, 0])
        self.T_1_2    = tf([0, 0, 0], [0, -np.pi/2, -np.pi/2])
        self.T_2_3    = tf([0.373, 0, 0], [0, 0, np.pi/2])
        self.T_3_4    = tf([0, -0.373, 0], [np.pi/2, 0, 0])
        self.T_4_5    = tf([0, 0, 0], [-np.pi/2, 0, 0])
        self.T_5_6    = tf([0, -0.1725, 0], [np.pi/2, 0, 0])

    def fk_e0509(self, joints_rad):
        """Computes flange position (mm) and ZYZ orientation (deg) matching Doosan posx format."""
        def rot_z(ang):
            c, s = math.cos(ang), math.sin(ang)
            return np.array([[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])

        T = (self.T_base_1 @ rot_z(joints_rad[0]) @
             self.T_1_2    @ rot_z(joints_rad[1]) @
             self.T_2_3    @ rot_z(joints_rad[2]) @
             self.T_3_4    @ rot_z(joints_rad[3]) @
             self.T_4_5    @ rot_z(joints_rad[4]) @
             self.T_5_6    @ rot_z(joints_rad[5]))

        pos_mm = T[:3, 3] * 1000.0
        r = R.from_matrix(T[:3, :3])
        # Doosan uses ZYZ euler angles formatted as [rz2, ry, rz1]
        try:
            zyz = r.as_euler('zyz', degrees=True)
            euler_deg = [zyz[2], zyz[1], zyz[0]]
        except Exception:
            euler_deg = [0.0, 180.0, 0.0]

        return pos_mm.tolist(), euler_deg

    def joint_state_cb(self, msg: JointState):
        name_map = {name: pos for name, pos in zip(msg.name, msg.position)}
        joints = []
        for i in range(1, 7):
            jname = f"joint_{i}"
            if jname in name_map:
                joints.append(name_map[jname])
        if len(joints) == 6:
            with self.lock:
                self.current_joints = joints
                self.joint_state_received = True

    # ── System Services ──────────────────────────────────────────
    def cb_set_robot_mode(self, request, response):
        mode_str = "MANUAL" if request.robot_mode == 0 else "AUTONOMOUS"
        self.get_logger().info(f"[Bridge] SetRobotMode({mode_str}) -> SUCCESS")
        response.success = True
        return response

    def cb_get_robot_mode(self, request, response):
        response.robot_mode = 1  # ROBOT_MODE_AUTONOMOUS
        return response

    def cb_get_robot_state(self, request, response):
        response.robot_state = 1  # STATE_STANDBY
        response.success = True
        return response

    def cb_get_current_posj(self, request, response):
        with self.lock:
            cur = list(self.current_joints)
        response.pos = [float(math.degrees(a)) for a in cur]
        response.success = True
        return response

    def cb_get_current_posx(self, request, response):
        with self.lock:
            cur = list(self.current_joints)
        pos_mm, euler_deg = self.fk_e0509(cur)
        sol = 2  # Standard forward solution
        task_data = [pos_mm[0], pos_mm[1], pos_mm[2], euler_deg[0], euler_deg[1], euler_deg[2], float(sol)]
        
        arr = Float64MultiArray()
        arr.data = task_data
        response.task_pos_info = [arr]
        response.success = True
        return response

    def cb_check_motion(self, request, response):
        with self.lock:
            response.status = 2 if self.is_moving else 0
        response.success = True
        return response

    # ── Gripper Service ──────────────────────────────────────────
    def cb_gripper_cmd(self, request, response):
        pos_input = float(request.position)
        # Position mapping: 0 (closed) -> 0.18 rad, 750 (open) -> -0.08 rad
        pos_rad = -0.08 + (750.0 - pos_input) / 750.0 * (0.18 - (-0.08))
        pos_rad = max(-0.08, min(0.20, pos_rad))

        state_str = "OPEN" if pos_input >= 500 else "CLOSE"
        self.get_logger().info(f"[Bridge] GripperCmd: {state_str} (input={pos_input:.0f} -> sim_joint={pos_rad:.3f} rad)")

        # Publish command to 4 symmetric joints in Gazebo
        msg = Float64MultiArray()
        msg.data = [pos_rad, pos_rad, pos_rad, pos_rad]
        steps = 10
        for _ in range(steps):
            self.pub_gripper.publish(msg)
            time.sleep(0.04)

        response.success = True
        return response

    # ── Motion Services ──────────────────────────────────────────
    def cb_move_joint(self, request, response):
        target_rad = [math.radians(deg) for deg in request.pos]
        self.get_logger().info(f"[Bridge] MoveJoint target(deg): {[round(d, 1) for d in request.pos]}")
        # Velocity in deg/s (default 20 deg/s)
        vel = request.vel if request.vel > 0 else 20.0
        with self.lock:
            cur = list(self.current_joints)
        max_diff = max(abs(t - c) for t, c in zip(target_rad, cur))
        duration = max(1.0, math.degrees(max_diff) / vel)

        self.execute_smooth_trajectory(target_rad, duration=duration)
        response.success = True
        return response

    def cb_move_line(self, request, response):
        x, y, z = request.pos[0], request.pos[1], request.pos[2]
        self.get_logger().info(f"[Bridge] MoveLine target: X={x:.1f}, Y={y:.1f}, Z={z:.1f} mm")
        target_rad = self.ik_e0509(x, y, z)
        vel = request.vel if request.vel > 0 else 30.0
        with self.lock:
            cur = list(self.current_joints)
        max_diff = max(abs(t - c) for t, c in zip(target_rad, cur))
        duration = max(1.2, math.degrees(max_diff) / 25.0)

        self.execute_smooth_trajectory(target_rad, duration=duration)
        response.success = True
        return response

    def cb_move_jointx(self, request, response):
        x, y, z = request.pos[0], request.pos[1], request.pos[2]
        self.get_logger().info(f"[Bridge] MoveJointx target: X={x:.1f}, Y={y:.1f}, Z={z:.1f} mm")
        target_rad = self.ik_e0509(x, y, z)
        vel = request.vel if request.vel > 0 else 20.0
        with self.lock:
            cur = list(self.current_joints)
        max_diff = max(abs(t - c) for t, c in zip(target_rad, cur))
        duration = max(1.2, math.degrees(max_diff) / vel)

        self.execute_smooth_trajectory(target_rad, duration=duration)
        response.success = True
        return response

    def ik_e0509(self, x_mm, y_mm, z_flange_mm):
        """
        Exact analytical IK for downward tool on e0509.
        Handles both flange coordinate and tool tip coordinates.
        """
        x = x_mm / 1000.0
        y = y_mm / 1000.0
        z_flange = z_flange_mm / 1000.0

        L1 = 0.373
        L2 = 0.373
        L3 = 0.1725
        z_base = 0.2045

        j1 = math.atan2(y, x)
        R_horiz = math.hypot(x, y)

        dR = R_horiz
        dZ = (z_flange + L3) - z_base

        D2 = dR**2 + dZ**2
        cos_j3 = (D2 - L1**2 - L2**2) / (2.0 * L1 * L2)
        cos_j3 = max(-1.0, min(1.0, cos_j3))
        j3 = math.acos(cos_j3)

        gamma = math.atan2(dR, dZ)
        psi = math.atan2(L2 * math.sin(j3), L1 + L2 * math.cos(j3))
        j2 = gamma - psi

        j4 = 0.0
        j5 = math.pi - (j2 + j3)
        j6 = 0.0

        return [j1, j2, j3, j4, j5, j6]

    def execute_smooth_trajectory(self, target_rad, duration=2.0, rate_hz=50):
        with self.lock:
            self.is_moving = True
            start_rad = list(self.current_joints)

        steps = max(10, int(duration * rate_hz))
        dt = 1.0 / rate_hz

        msg = Float64MultiArray()
        for i in range(steps + 1):
            t = i / float(steps)
            # S-curve interpolation
            s = (1.0 - math.cos(t * math.pi)) / 2.0
            interp = [start + s * (goal - start) for start, goal in zip(start_rad, target_rad)]
            msg.data = interp
            self.pub_arm.publish(msg)
            time.sleep(dt)

        with self.lock:
            self.current_joints = list(target_rad)
            self.is_moving = False


def main(args=None):
    rclpy.init(args=args)
    bridge = DsrSimBridge()
    try:
        rclpy.spin(bridge)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        bridge.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

