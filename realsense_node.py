#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RealSense D400 ROS 2 Publisher Node
- Uses pyrealsense2 to capture Color & Depth (Aligned).
- Publishes standard ROS 2 Image and CameraInfo topics.
- Provides compatibility topics (/camera/image, /camera/depth_image, /camera/camera_info).
"""

import sys
import time
import numpy as np
import pyrealsense2 as rs

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge


class RealSenseNode(Node):
    def __init__(self):
        super().__init__('realsense_node')
        self.bridge = CvBridge()

        # Declare parameters
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 30)

        width = self.get_parameter('width').get_parameter_value().integer_value
        height = self.get_parameter('height').get_parameter_value().integer_value
        fps = self.get_parameter('fps').get_parameter_value().integer_value

        self.declare_parameter('compat_topics', False)
        self.compat_topics = self.get_parameter('compat_topics').get_parameter_value().bool_value

        # Publishers (Standard realsense2_camera topics)
        self.pub_color = self.create_publisher(Image, '/camera/camera/color/image_raw', 10)
        self.pub_depth = self.create_publisher(Image, '/camera/camera/aligned_depth_to_color/image_raw', 10)
        self.pub_info = self.create_publisher(CameraInfo, '/camera/camera/color/camera_info', 10)

        # Compatibility topics (optional, default off to avoid clashing with Gazebo)
        if self.compat_topics:
            self.pub_compat_color = self.create_publisher(Image, '/camera/image', 10)
            self.pub_compat_depth = self.create_publisher(Image, '/camera/depth_image', 10)
            self.pub_compat_info = self.create_publisher(CameraInfo, '/camera/camera_info', 10)
        else:
            self.pub_compat_color = None
            self.pub_compat_depth = None
            self.pub_compat_info = None

        self.get_logger().info(f"Initializing Intel RealSense Pipeline ({width}x{height} @ {fps}fps)...")
        self.pipeline = rs.pipeline()
        self.config = rs.config()

        self.config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
        self.config.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)

        try:
            self.profile = self.pipeline.start(self.config)
        except Exception as e:
            self.get_logger().error(f"Failed to start RealSense camera pipeline: {e}")
            sys.exit(1)

        self.align = rs.align(rs.stream.color)
        color_stream = self.profile.get_stream(rs.stream.color).as_video_stream_profile()
        self.intrinsics = color_stream.get_intrinsics()

        # Warm up exposure
        for _ in range(15):
            self.pipeline.wait_for_frames()

        self.timer = self.create_timer(1.0 / fps, self.timer_callback)
        self.get_logger().info("Intel RealSense D455 successfully streaming!")
        self.get_logger().info(f"  Intrinsics: fx={self.intrinsics.fx:.2f}, fy={self.intrinsics.fy:.2f}, "
                              f"cx={self.intrinsics.ppx:.2f}, cy={self.intrinsics.ppy:.2f}")

    def _reconnect(self):
        self.get_logger().info("Attempting RealSense reconnection...")
        try:
            self.pipeline.stop()
        except Exception:
            pass
        time.sleep(1.0)
        try:
            self.profile = self.pipeline.start(self.config)
            self.align = rs.align(rs.stream.color)
            self.get_logger().info("RealSense reconnected successfully.")
        except Exception as e:
            self.get_logger().warn(f"Reconnection retry failed: {e}", throttle_duration_sec=2.0)

    def timer_callback(self):
        if not rclpy.ok():
            return
        try:
            frames = self.pipeline.wait_for_frames(timeout_ms=1000)
        except Exception as e:
            self.get_logger().warn(f"Frame dropped or timeout: {e}", throttle_duration_sec=2.0)
            if "wait_for_frames cannot be called before start" in str(e) or "disconnected" in str(e).lower():
                self._reconnect()
            return

        aligned_frames = self.align.process(frames)
        color_frame = aligned_frames.get_color_frame()
        depth_frame = aligned_frames.get_depth_frame()

        if not color_frame or not depth_frame:
            return

        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())

        if not rclpy.ok():
            return

        now = self.get_clock().now().to_msg()

        # CameraInfo Message
        info_msg = CameraInfo()
        info_msg.header.stamp = now
        info_msg.header.frame_id = 'camera_color_optical_frame'
        info_msg.width = self.intrinsics.width
        info_msg.height = self.intrinsics.height
        info_msg.distortion_model = 'plumb_bob'
        info_msg.d = list(self.intrinsics.coeffs)
        info_msg.k = [
            self.intrinsics.fx, 0.0, self.intrinsics.ppx,
            0.0, self.intrinsics.fy, self.intrinsics.ppy,
            0.0, 0.0, 1.0
        ]
        info_msg.p = [
            self.intrinsics.fx, 0.0, self.intrinsics.ppx, 0.0,
            0.0, self.intrinsics.fy, self.intrinsics.ppy, 0.0,
            0.0, 0.0, 1.0, 0.0
        ]

        # ROS Images (direct fast conversion, bypasses cv_bridge Jazzy bug)
        color_msg = Image()
        color_msg.header.stamp = now
        color_msg.header.frame_id = 'camera_color_optical_frame'
        color_msg.height = color_image.shape[0]
        color_msg.width = color_image.shape[1]
        color_msg.encoding = 'bgr8'
        color_msg.is_bigendian = 0
        color_msg.step = color_image.shape[1] * 3
        color_msg.data = color_image.tobytes()

        depth_msg = Image()
        depth_msg.header.stamp = now
        depth_msg.header.frame_id = 'camera_color_optical_frame'
        depth_msg.height = depth_image.shape[0]
        depth_msg.width = depth_image.shape[1]
        depth_msg.encoding = '16UC1'
        depth_msg.is_bigendian = 0
        depth_msg.step = depth_image.shape[1] * 2
        depth_msg.data = depth_image.tobytes()

        try:
            # Publish to primary topics
            self.pub_color.publish(color_msg)
            self.pub_depth.publish(depth_msg)
            self.pub_info.publish(info_msg)

            # Publish to compatibility topics (if enabled)
            if self.compat_topics:
                self.pub_compat_color.publish(color_msg)
                self.pub_compat_depth.publish(depth_msg)
                self.pub_compat_info.publish(info_msg)
        except Exception:
            pass

    def destroy_node(self):
        self.get_logger().info("Stopping RealSense pipeline...")
        try:
            self.timer.cancel()
        except Exception:
            pass
        try:
            self.pipeline.stop()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RealSenseNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
