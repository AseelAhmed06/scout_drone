#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, HistoryPolicy
import os
import csv
import cv2
import numpy as np
from collections import deque
from sensor_msgs.msg import NavSatFix, Imu, Image # Image is not directly used but kept for completeness as it was in original
from cv_bridge import CvBridge # CvBridge is not directly used but kept for completeness as it was in original
from std_msgs.msg import String


class Geotagger(Node):
    """
    ROS 2 Geotagger Node for associating GPS and IMU data with image timestamps.

    This node subscribes to:
    - '/camera/timestamps': String messages containing image timestamps and filenames.
    - '/mavros/imu/data': Imu messages for orientation data.
    - '/mavros/global_position/global': NavSatFix messages for global position.

    It interpolates GPS and IMU data to match the image timestamps and saves
    the combined geotagging information (timestamp, lat, lon, alt, orientation_w, yaw, filename)
    to a CSV file.
    """
    def __init__(self):
        """
        Initializes the Geotagger node, sets up data buffers, file paths,
        and ROS 2 publishers/subscribers.
        """
        super().__init__('geotagger_node')

        # Data buffers to store incoming sensor messages.
        # deque (double-ended queue) is used for efficient appending and
        # automatic removal of old data based on maxlen.
        self.gps_buffer = deque(maxlen=200)
        self.imu_buffer = deque(maxlen=300)
        # The original image_buffer had a maxlen=1 and was cleared immediately.
        # This implementation processes image metadata directly upon reception,
        # removing the need for a separate image_buffer.

        self.bridge = CvBridge()  # CvBridge instance (not directly used for image content in this version)
        self.image_count = 0      # Counter for processed images

        # Define the directory to save output data.
        self.declare_parameter('folder_path', '/home/aseel/')
        self.save_dir = self.get_parameter('folder_path').get_parameter_value().string_value

        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)
            self.get_logger().info(f"Created output directory: {self.save_dir}")

        # Define the CSV file path and write the header row.
        # 'w' mode with newline='' is used for proper CSV handling in Python 3.
        self.csv_file = os.path.join(self.save_dir, 'geotag_data.csv')
        try:
            with open(self.csv_file, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['timestamp', 'latitude', 'longitude', 'altitude', 'orientation_w', 'yaw', 'filename'])
            self.get_logger().info(f"Initialized CSV file: {self.csv_file}")
        except IOError as e:
            self.get_logger().error(f"Failed to open or write to CSV file {self.csv_file}: {e}")

        # ROS 2 QoS Profiles:
        # qos_profile_sensor_data is suitable for sensor topics where
        # occasional message loss is acceptable for real-time performance.
        sensor_qos = qos_profile_sensor_data
        # For status messages, a more reliable QoS might be preferred to ensure delivery.
        status_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # ROS 2 Subscribers:
        # Each subscription specifies the message type, topic name, callback function, and QoS profile.
        self.create_subscription(String, '/camera/timestamps', self.image_callback, sensor_qos)
        self.create_subscription(Imu, '/mavros/imu/data', self.imu_callback, sensor_qos)
        self.create_subscription(NavSatFix, '/mavros/global_position/global', self.gps_callback, sensor_qos)
        self.geotag_pub = self.create_publisher(String, '/geotag_data', status_qos)

        # ROS 2 Publisher for status updates:
        self.status_pub = self.create_publisher(String, '/geotagger/status', status_qos)

        self.get_logger().info('ROS 2 Geotagger node started.')
        self.status_pub.publish(String(data="active")) # Publish initial status

    def gps_callback(self, msg):
        """
        Callback for NavSatFix messages. Appends the message timestamp and data to the GPS buffer.
        """
        # Convert ROS 2 Time object to seconds for consistency with interpolation logic.
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        self.gps_buffer.append((timestamp, msg))

    def imu_callback(self, msg):
        """
        Callback for Imu messages. Appends the current ROS time and data to the IMU buffer.
        """
        # Get current ROS 2 time.
        timestamp = self.get_clock().now().to_msg().sec + self.get_clock().now().to_msg().nanosec / 1e9
        self.imu_buffer.append((timestamp, msg))

    def image_callback(self, msg):
        """
        Callback for image timestamp messages.
        Parses the image timestamp and filename, then triggers processing.
        """
        try:
            timestamp_str, filename = msg.data.split(',')
            image_time = float(timestamp_str)
            self.get_logger().info(f"Received image metadata: time={image_time}, file={filename}")
            # Directly process the received image metadata
            self.process_image_metadata(image_time, filename)
        except ValueError as e:
            self.get_logger().error(f"Failed to parse image metadata string '{msg.data}': {e}")
        except Exception as e:
            self.get_logger().error(f"An unexpected error occurred in image_callback: {e}")

    def process_image_metadata(self, image_time, image_filename):
        """
        Processes a single image's metadata by interpolating corresponding GPS and IMU data.
        Saves the combined data to the CSV file.
        """
        self.get_logger().info(f"Processing metadata with timestamp: {image_time}")

        # Interpolate GPS data (latitude, longitude, altitude) for the image timestamp.
        gps_interp = self.interpolate(self.gps_buffer, image_time, ['latitude', 'longitude', 'altitude'])
        # Interpolate IMU quaternion for the image timestamp.
        quat = self.interpolate_quaternion(self.imu_buffer, image_time)

        # If interpolation fails for either GPS or IMU, log a warning and skip this frame.
        if gps_interp is None or quat is None:
            self.status_pub.publish(String(data="interpolation failed"))
            self.get_logger().warn(f"Interpolation failed for image at {image_time}, skipping frame.")
            return

        # Extract interpolated GPS values.
        lat = gps_interp['latitude']
        lon = gps_interp['longitude']
        alt = gps_interp['altitude']

        # Extract yaw from the interpolated quaternion.
        yaw = self.extract_yaw_from_quaternion(quat)
        # Placeholder for Kalman filter. If not used, 'filtered_yaw' can be removed
        # and 'yaw' directly used.
        filtered_yaw = yaw

        try:
            # Open the CSV file in append mode ('a') with newline='' for Python 3.
            with open(self.csv_file, 'a', newline='') as f:
                writer = csv.writer(f)
                # Write the geotagged data row.
                writer.writerow([
                    image_time,
                    lat,
                    lon,
                    alt,
                    quat[3],  # orientation_w component of the quaternion (x, y, z, w)
                    filtered_yaw,
                    image_filename
                ])
            geotag_msg = f"{image_time},{lat},{lon},{alt},{quat[3]},{filtered_yaw},{image_filename}"
            self.geotag_pub.publish(String(data=geotag_msg))

            self.get_logger().info(f"Saved metadata for file: {image_filename}")
            self.status_pub.publish(String(data="geotagging")) # Update status
            self.image_count += 1

        except IOError as e:
            self.get_logger().error(f"Failed to write to CSV file: {e}")
        except Exception as e:
            self.get_logger().error(f"An unexpected error occurred while saving metadata: {e}")

    def interpolate(self, buffer, target_time, fields):
        """
        Performs linear interpolation on sensor data (e.g., GPS) based on timestamps.

        Args:
            buffer (deque): A deque of (timestamp, message_object) tuples.
            target_time (float): The timestamp for which to interpolate data.
            fields (list): A list of attribute names (strings) to interpolate from the message objects.

        Returns:
            dict or None: A dictionary containing interpolated values for the specified fields,
                          or None if interpolation is not possible (e.g., buffer too small,
                          target_time outside buffer range).
        """
        lower = None
        upper = None

        # Find the two data points that bracket the target_time.
        for i in range(len(buffer) - 1):
            if buffer[i][0] <= target_time < buffer[i+1][0]:
                lower = buffer[i]
                upper = buffer[i+1]
                break

        if lower is None or upper is None:
            # If target_time is outside the buffer range, try to use the closest point.
            if len(buffer) > 0:
                if target_time < buffer[0][0]: # Target is before the first timestamp
                    lower = buffer[0]
                    upper = buffer[0]
                    self.get_logger().warn(f"Target time {target_time} is before first buffer timestamp {buffer[0][0]}. Using earliest data.")
                elif target_time >= buffer[-1][0]: # Target is after the last timestamp
                    lower = buffer[-1]
                    upper = buffer[-1]
                    self.get_logger().warn(f"Target time {target_time} is after last buffer timestamp {buffer[-1][0]}. Using latest data.")
            else:
                self.get_logger().warn(f"Buffer is empty for interpolation at time {target_time}.")
                return None

        # If still no suitable points, return None.
        if lower is None or upper is None:
            self.get_logger().warn(f"Could not find suitable points for interpolation at time {target_time}.")
            return None

        interpolated_data = {}
        # Handle the case where target_time is exactly at lower or upper, or only one point is available.
        # This avoids division by zero if lower and upper have the same timestamp.
        if lower[0] == upper[0]:
            for field in fields:
                interpolated_data[field] = getattr(lower[1], field)
            return interpolated_data

        # Perform linear interpolation for each specified field.
        for field in fields:
            lower_value = getattr(lower[1], field)
            upper_value = getattr(upper[1], field)
            time_diff = upper[0] - lower[0]

            # This check is redundant if lower[0] == upper[0] is handled above,
            # but kept for robustness against floating-point precision issues causing a tiny non-zero diff.
            if time_diff == 0:
                self.get_logger().warn(f"Zero time difference in interpolation for field '{field}'. Using lower value.")
                interpolated_data[field] = lower_value
                continue

            weight_upper = (target_time - lower[0]) / time_diff
            weight_lower = 1 - weight_upper
            interpolated_data[field] = lower_value * weight_lower + upper_value * weight_upper

        return interpolated_data

    def interpolate_quaternion(self, buffer, target_time):
        """
        Performs linear interpolation of quaternions (slerp is more accurate but complex,
        linear interpolation followed by normalization is a common approximation for small time differences).

        Args:
            buffer (deque): A deque of (timestamp, Imu_message) tuples.
            target_time (float): The timestamp for which to interpolate the quaternion.

        Returns:
            np.array or None: A 4-element numpy array representing the interpolated quaternion [x, y, z, w],
                              or None if interpolation is not possible.
        """
        lower = None
        upper = None

        # Find the two data points that bracket the target_time.
        for i in range(len(buffer) - 1):
            if buffer[i][0] <= target_time < buffer[i+1][0]:
                lower = buffer[i]
                upper = buffer[i+1]
                break

        if lower is None or upper is None:
            # If target_time is outside the buffer range, try to use the closest point.
            if len(buffer) > 0:
                if target_time < buffer[0][0]:
                    lower = buffer[0]
                    upper = buffer[0]
                    self.get_logger().warn(f"Target time {target_time} is before first IMU timestamp {buffer[0][0]}. Using earliest data.")
                elif target_time >= buffer[-1][0]:
                    lower = buffer[-1]
                    upper = buffer[-1]
                    self.get_logger().warn(f"Target time {target_time} is after last IMU timestamp {buffer[-1][0]}. Using latest data.")
            else:
                self.get_logger().warn(f"IMU buffer is empty for quaternion interpolation at time {target_time}.")
                return None

        # If still no suitable points, return None.
        if lower is None or upper is None:
            self.get_logger().warn(f"Could not find suitable IMU points for quaternion interpolation at time {target_time}.")
            return None

        # Handle the case where target_time is exactly at lower or upper, or only one point is available.
        if lower[0] == upper[0]:
            return self.quaternion_to_array(lower[1].orientation)

        q_lower = lower[1].orientation
        q_upper = upper[1].orientation
        time_diff = upper[0] - lower[0]

        if time_diff == 0:
            self.get_logger().warn("Zero time difference in quaternion interpolation. Using lower quaternion.")
            return self.quaternion_to_array(q_lower)

        weight_upper = (target_time - lower[0]) / time_diff
        weight_lower = 1 - weight_upper

        q_lower_array = self.quaternion_to_array(q_lower)
        q_upper_array = self.quaternion_to_array(q_upper)

        # Linear interpolation of quaternions.
        q_interp = np.multiply(q_lower_array, weight_lower) + np.multiply(q_upper_array, weight_upper)

        # Normalize the interpolated quaternion to ensure it remains a unit quaternion.
        norm = np.linalg.norm(q_interp)
        if norm == 0:
            self.get_logger().warn("Interpolated quaternion has zero norm, cannot normalize. Returning original lower quaternion.")
            return q_lower_array
        q_interp = q_interp / norm

        return q_interp

    def quaternion_to_array(self, q):
        """Converts a ROS Quaternion message object to a numpy array [x, y, z, w]."""
        return np.array([q.x, q.y, q.z, q.w])

    def extract_yaw_from_quaternion(self, quat):
        """
        Extracts the yaw (z-axis rotation) from a quaternion.

        Args:
            quat (np.array): A 4-element numpy array representing the quaternion [x, y, z, w].

        Returns:
            float: The yaw angle in radians.
        """
        x, y, z, w = quat
        # Ensure the quaternion is normalized before converting to Euler angles.
        norm = np.linalg.norm(quat)
        if norm == 0:
            self.get_logger().warn("Attempted to extract yaw from a zero-norm quaternion.")
            return 0.0
        # Normalize in case it was not normalized earlier
        x, y, z, w = x/norm, y/norm, z/norm, w/norm

        _, _, yaw = self.euler_from_quaternion([x, y, z, w])
        return yaw

    def euler_from_quaternion(self, quat):
        """
        Convert a quaternion into Euler angles (roll, pitch, yaw).
        Based on https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
        This function is for standard Aerospace sequence (yaw, pitch, roll) (Z-Y-X).

        Args:
            quat (list or np.array): A list or numpy array [x, y, z, w] representing the quaternion.

        Returns:
            tuple: (roll_x, pitch_y, yaw_z) in radians.
        """
        x, y, z, w = quat

        # Roll (x-axis rotation)
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = np.arctan2(t0, t1)

        # Pitch (y-axis rotation)
        t2 = +2.0 * (w * y - z * x)
        # Clip t2 to ensure it's within [-1.0, 1.0] due to floating point inaccuracies
        t2 = np.clip(t2, -1.0, 1.0)
        pitch_y = np.arcsin(t2)

        # Yaw (z-axis rotation)
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = np.arctan2(t3, t4)

        return roll_x, pitch_y, yaw_z


def main(args=None):
    """
    Main function to initialize and run the ROS 2 Geotagger node.
    """
    rclpy.init(args=args)      # Initialize ROS 2 client library
    geotagger = Geotagger()    # Create an instance of the Geotagger node
    rclpy.spin(geotagger)      # Keep the node alive until interrupted
    geotagger.destroy_node()   # Clean up node resources
    rclpy.shutdown()           # Shutdown ROS 2 client library

if __name__ == '__main__':
    main()
