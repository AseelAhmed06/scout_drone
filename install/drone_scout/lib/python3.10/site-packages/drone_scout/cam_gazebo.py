#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import cv2
import os
from datetime import datetime
from mavros_msgs.msg import WaypointReached
class GStreamerImageSaver(Node):
    def __init__(self):
        super().__init__('gstream_image_saver')

        # Declare and get parameters
        self.declare_parameter("folder_path", "/home/aseel/gazebo")
        self.output_dir = self.get_parameter("folder_path").get_parameter_value().string_value
        self.output_dir = os.path.join(self.output_dir, "captured_frames")
        os.makedirs(self.output_dir, exist_ok=True)
        self.declare_parameter('start_waypoint_index', 1)
        self.declare_parameter('stop_waypoint_index', 7)
        self.start_waypoint_index = self.get_parameter('start_waypoint_index').get_parameter_value().integer_value
        self.stop_waypoint_index = self.get_parameter('stop_waypoint_index').get_parameter_value().integer_value
        self.publisher_ = self.create_publisher(String, "/camera/timestamps", 10)
        self.run = False
        self.get_logger().info(f"Saving images to: {self.output_dir}")
        self.get_logger().info("Publishing metadata to: udp_image_metadata")
        self.create_subscription(WaypointReached, '/mavros/mission/reached', self.waypoint_callback, 10)
        self.current_waypoint_index = -1
        # Open GStreamer pipeline using OpenCV
        self.pipeline = (
            "udpsrc port=5600 caps=application/x-rtp,media=video,clock-rate=90000,encoding-name=H264 ! "
            "rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! appsink"
        )
        self.cap = cv2.VideoCapture(self.pipeline, cv2.CAP_GSTREAMER)

        if not self.cap.isOpened():
            self.get_logger().error("Failed to open GStreamer pipeline.")
            return

        # Timer to poll frames
        self.timer = self.create_timer(1.0, self.capture_frame)  # ~1 Hz

    def waypoint_callback(self, msg):
        new_waypoint_index = msg.wp_seq

        if new_waypoint_index != self.current_waypoint_index:
            old_wp = self.current_waypoint_index
            self.current_waypoint_index = new_waypoint_index

            self.get_logger().info(f"Waypoint changed from {old_wp} to {new_waypoint_index}.")

            if new_waypoint_index == self.start_waypoint_index:
                self.get_logger().info(f"Detected START WAYPOINT ({new_waypoint_index}). Starting image capture.")
                self.run = True

            elif new_waypoint_index == self.stop_waypoint_index:
                self.get_logger().info(f"Detected STOP WAYPOINT ({new_waypoint_index}). Stopping image capture.")
                self.run=False

            else:
                self.get_logger().info(f"Currently at waypoint {new_waypoint_index}. Waiting for start ({self.start_waypoint_index}) or stop ({self.stop_waypoint_index}).")

    def capture_frame(self):
        if not self.run:
            return
        
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("No frame received.")
            return

        timestamp = self.get_clock().now().nanoseconds
        image_filename = f"frame_{timestamp}.jpg"
        image_path = os.path.join(self.output_dir, image_filename)

        cv2.imwrite(image_path, frame)
        self.get_logger().info(f"Saved: {image_filename}")

        # Publish metadata
        msg = String()
        msg.data = f"{timestamp},{image_filename}"
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published metadata: {msg.data}")


def main(args=None):
    rclpy.init(args=args)
    node = GStreamerImageSaver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
