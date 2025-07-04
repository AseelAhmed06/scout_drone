#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import Header, String
from sensor_msgs.msg import Imu, NavSatFix, TimeReference
from mavros_msgs.msg import State
import json
import subprocess # For calling ros2 node list
from mavros_msgs.msg import WaypointReached

class DroneStatusMonitor(Node):
    """
    A ROS2 node to monitor the status of various drone components and publish
    a consolidated status message.
    """

    def __init__(self):
        # Initialize the ROS2 node with the name 'drone_status_monitor'
        super().__init__('drone_status_monitor')
        self.get_logger().info('Drone Status Monitor Node Initialized.')

        # Initialize status variables
        self.mavros_connected = False
        self.imu_available = False
        self.gps_available = False
        self.geotagger_status = "unknown"
        self.camera_status = "Initializing..."
        self.fcu_time = None # Store FCU time as a builtin_interfaces/Time object
        self.waypoint = 0
        # Define QoS profiles for subscriptions and publications
        # QoS for sensor data (IMU, GPS) - best effort, small history
        qos_profile_sensor_data = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        # QoS for general system status - reliable, larger history
        qos_profile_system_default = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Create Subscribers
        self.state_sub = self.create_subscription(
            State,
            '/mavros/state',
            self.state_cb,
            qos_profile_system_default
        )
        self.imu_sub = self.create_subscription(
            Imu,
            '/mavros/imu/data',
            self.imu_cb,
            qos_profile_sensor_data
        )
        self.gps_sub = self.create_subscription(
            NavSatFix,
            '/mavros/global_position/global',
            self.gps_cb,
            qos_profile_sensor_data
        )
        self.geotagger_status_sub = self.create_subscription(
            String,
            '/geotagger/status',
            self.geotagger_status_cb,
            qos_profile_system_default
        )
        self.time_reference_sub = self.create_subscription(
            TimeReference,
            '/mavros/time_reference',
            self.time_reference_cb,
            qos_profile_system_default
        )
        self.camera_status_sub = self.create_subscription(
            String,
            '/camera/status',
            self.camera_status_cb,
            qos_profile_system_default
        )

        # Create Publisher for drone status
        self.status_pub = self.create_publisher(
            String,
            '/drone_status',
            qos_profile_system_default
        )
        self.create_subscription(WaypointReached, '/mavros/mission/reached', self.waypoint_callback, 10)
        # Create a timer to periodically check and publish status
        # The timer calls status_check every 1.0 second
        self.timer = self.create_timer(1.0, self.status_check)

    def state_cb(self, msg: State):
        """Callback for MAVROS state messages."""
        self.mavros_connected = msg.connected

    def waypoint_callback(self, msg):
        self.waypoint = msg.wp_seq

    def time_reference_cb(self, msg: TimeReference):
        """Callback for time reference messages from MAVROS."""
        # Store the time_ref field, which is a builtin_interfaces/Time object
        self.fcu_time = msg.time_ref

    def imu_cb(self, msg: Imu):
        """Callback for IMU data messages."""
        self.imu_available = True

    def gps_cb(self, msg: NavSatFix):
        """Callback for GPS data messages."""
        # Check if GPS status is valid (typically >= 0 for a fix)
        if msg.status.status >= 0:
            self.gps_available = True
        else:
            self.gps_available = False

    def geotagger_status_cb(self, msg: String):
        """Callback for geotagger status messages."""
        self.geotagger_status = msg.data

    def camera_status_cb(self, msg: String):
        """Callback for camera status messages."""
        self.camera_status = msg.data

    def status_check(self):
        """
        Periodically checks the status of various components, publishes the
        consolidated status, and prints it to the console.
        """
        mavros_running = False
        geotagger_running = False
        camera_node_running = False
        inference_running = False
        waypoint_generator = False

        try:
            # Get a list of running ROS2 nodes using subprocess
            # This mimics rosnode.get_node_names() functionality in ROS1
            # Note: This method can be slow and is not ideal for real-time systems.
            # In ROS2, node management is often handled via launch files.
            result = subprocess.check_output(['ros2', 'node', 'list'], text=True)
            running_nodes = result.strip().split('\n')

            mavros_running = any('/mavros' in node for node in running_nodes)
            geotagger_running = any('/geotag' in node or '/geotagger_node' in node for node in running_nodes)
            camera_node_running = any('/cam_csi' in node or '/cam_gazebo' in node or '/cam_test' in node for node in running_nodes)
            inference_running = any('/inference' in node for node in running_nodes)
            waypoint_generator =  any('/waypoint_generate' in node  for node in running_nodes)
        except Exception as e:
            self.get_logger().error(f"Failed to get running nodes: {e}")
            # Keep running_status as False if an error occurs

        # Get current ROS2 time
        ros_time_obj = self.get_clock().now()
        ros_time_sec = ros_time_obj.nanoseconds / 1e9 # Convert nanoseconds to seconds

        fcu_time_str = "N/A"
        if self.fcu_time:
            # Convert builtin_interfaces/Time object to seconds
            fcu_time_sec = self.fcu_time.sec + self.fcu_time.nanosec / 1e9
            fcu_time_str = "{:.9f}".format(fcu_time_sec)

        # Create a dictionary of the current status
        status_dict = {
            "ros_time_sec": ros_time_sec,
            "fcu_time_sec": fcu_time_str,
            "mavros_running": mavros_running,
            "fcu_connected": self.mavros_connected,
            "imu_available": self.imu_available,
            "gps_available": self.gps_available,
            "waypoint_reached":str(self.waypoint),
            "geotagger_running": geotagger_running,
            "geotagger_status": self.geotagger_status,
            "camera_node_running": camera_node_running,
            "camera_status": self.camera_status,
            "waypoint_generate": waypoint_generator,
            "infernce_running":inference_running
        }

        # Publish the status as a JSON string
        status_msg = String()
        status_msg.data = json.dumps(status_dict)
        self.status_pub.publish(status_msg)

        # Print status to console for readability
        print("\n" + "="*30)
        print("[DRONE STATUS MONITOR]")
        print("="*30)
        print("ROS Time (sec):           {:.9f}".format(ros_time_sec))
        print("FCU Time (sec):           {}".format(fcu_time_str))
        print("-" * 30)
        print("MAVROS node running:      {}".format("running" if mavros_running else "NOT RUNNING"))
        print("FCU connected:            {}".format("CONNECTED" if self.mavros_connected else "NOT CONNECTED"))
        print("IMU data received:        {}".format("data available" if self.imu_available else "NO DATA"))
        print("GPS data received:        {}".format("data available" if self.gps_available else "NO FIX"))
        print("Waypoint Reached:        {}".format(str(self.waypoint)))
        print("-" * 30)
        print("Image Geotagger running:  {}".format("running" if geotagger_running else "NOT RUNNING"))
        print("Image Geotagger status:   {}".format(self.geotagger_status))
        print("-" * 30)
        print("Camera Node running:      {}".format("running" if camera_node_running else "NOT RUNNING"))
        print("Camera Status:            {}".format(self.camera_status))
        print("-" * 30)
        print("Infer Node running:      {}".format("running" if inference_running else "NOT RUNNING"))
        print("-" * 30)
        print("Waypoint Node running:      {}".format("running" if waypoint_generator else "NOT RUNNING"))
        print("="*30 + "\n")

def main(args=None):
    """Main function to initialize and run the ROS2 node."""
    rclpy.init(args=args) # Initialize ROS2 client library
    node = DroneStatusMonitor() # Create an instance of the node

    try:
        rclpy.spin(node) # Keep the node alive until interrupted
    except KeyboardInterrupt:
        node.get_logger().info('Drone Status Monitor Node stopped by user.')
    finally:
        node.destroy_node() # Cleanly destroy the node
        rclpy.shutdown() # Shutdown ROS2 client library

if __name__ == '__main__':
    main()
