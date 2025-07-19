#!/usr/bin/env python3
import os
import re
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from mavros_msgs.msg import WaypointReached
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import shutil

class CamTest(Node):
    def __init__(self):
        super().__init__('cam_test')

        # Declare parameters
        self.declare_parameter('image_folder', '/home/aseel/images')
        self.declare_parameter('start_waypoint_index', 1)
        self.declare_parameter('stop_waypoint_index', 7)
        self.declare_parameter('folder_path', '/home/aseel/copied_images')
        self.destination_folder = self.get_parameter('folder_path').get_parameter_value().string_value 
        self.destination_folder = os.path.join(self.destination_folder, "captured_frames")
        # Create the image save directory if it does not exist
        os.makedirs(self.destination_folder, exist_ok=True)
        self.image_folder = self.get_parameter('image_folder').get_parameter_value().string_value
        self.start_waypoint_index = self.get_parameter('start_waypoint_index').get_parameter_value().integer_value
        self.stop_waypoint_index = self.get_parameter('stop_waypoint_index').get_parameter_value().integer_value
        qos_profile_system_default = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        # Regex to match filenames like nvcamtest_8131_s00_00008.jpg
        self.image_pattern = re.compile(r'nvcamtest_\d+_s\d+_(\d+)\.(jpg|jpeg|png|bmp)', re.IGNORECASE)

        # Publishers and subscribers
        self.timestamp_publisher = self.create_publisher(String, '/camera/timestamps', 10)
        self.create_subscription(WaypointReached, '/mavros/mission/reached', self.waypoint_callback, 10)

        self.image_list = []
        self.image_index = 0
        self.active = False
        self.timer = None
        self.current_waypoint_index = -1
        self.create_subscription(
            String,
            '/drone_status/last_waypoint_index',
            self.waypoint_index_callback,
            qos_profile_system_default
        )
        self.get_logger().info('ImageTimestampPublisher initialized and waiting for waypoints...')

    def waypoint_index_callback(self, msg: String):
        """
        Callback function for the /drone_status/last_waypoint_index topic.
        Converts the received string data to an integer.
        """
        try:
           
            last_waypoint_index_int = int(msg.data)
            self.get_logger().info(f'Received last waypoint index (int): {last_waypoint_index_int}')
            self.stop_waypoint_index = last_waypoint_index_int
        except ValueError:
            self.get_logger().error(f'Failed to convert received data "{msg.data}" to integer. Is the publisher sending valid integer strings?')
        except Exception as e:
            self.get_logger().error(f'An unexpected error occurred: {e}')


    def waypoint_callback(self, msg):
        new_waypoint_index = msg.wp_seq

        if new_waypoint_index != self.current_waypoint_index:
            old_wp = self.current_waypoint_index
            self.current_waypoint_index = new_waypoint_index

            self.get_logger().info(f"Waypoint changed from {old_wp} to {new_waypoint_index}.")

            if new_waypoint_index == self.start_waypoint_index:
                self.get_logger().info(f"Detected START WAYPOINT ({new_waypoint_index}). Starting image capture.")
                self.start_capture()

            elif new_waypoint_index == self.stop_waypoint_index:
                self.get_logger().info(f"Detected STOP WAYPOINT ({new_waypoint_index}). Stopping image capture.")
                self.stop_capture()

            else:
                self.get_logger().info(f"Currently at waypoint {new_waypoint_index}. Waiting for start ({self.start_waypoint_index}) or stop ({self.stop_waypoint_index}).")

    def start_capture(self):
        self.load_images()
        if self.image_list:
            self.image_index = 0
            self.active = True
            self.timer = self.create_timer(1.0, self.publish_next_image)
        else:
            self.get_logger().warn("No matching images found in the folder.")

    def stop_capture(self):
        if self.timer:
            self.timer.cancel()
        self.active = False
        self.get_logger().info("Image capture stopped.")

    def load_images(self):
        if not os.path.isdir(self.image_folder):
            self.get_logger().error(f"Invalid image folder: {self.image_folder}")
            return

        files = os.listdir(self.image_folder)
        matched_files = [f for f in files if self.image_pattern.match(f)]
        sorted_files = sorted(matched_files, key=lambda f: int(self.image_pattern.match(f).group(1)))
        self.image_list = sorted_files
        self.get_logger().info(f"Loaded {len(self.image_list)} images for publishing.")

    def publish_next_image(self):
        if self.image_index < len(self.image_list):
            image_filename = self.image_list[self.image_index]
            timestamp_ns = self.get_clock().now().nanoseconds
            msg = String()
            msg.data = f"{timestamp_ns},{image_filename}"
            self.timestamp_publisher.publish(msg)
            self.get_logger().info(f"Published: {msg.data}")

            source_path = os.path.join(self.image_folder, image_filename)
            destination_path = os.path.join(self.destination_folder, image_filename)
            try:
                # shutil.copy2 preserves metadata (like creation/modification times)
                shutil.copy2(source_path, destination_path)
                self.get_logger().info(f"Copied image from {source_path} to {destination_path}")
            except FileNotFoundError:
                self.get_logger().error(f"Error copying image: Source file not found at {source_path}")
            except PermissionError:
                self.get_logger().error(f"Error copying image: Permission denied to write to {self.destination_folder}")
            except Exception as e:
                self.get_logger().error(f"An unexpected error occurred while copying {image_filename}: {e}")

            self.image_index += 1
        else:
            self.get_logger().info("All image timestamps published.")
            self.stop_capture()

def main(args=None):
    rclpy.init(args=args)
    node = CamTest()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
