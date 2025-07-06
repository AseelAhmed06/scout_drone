import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy, qos_profile_sensor_data
from rclpy.logging import set_logger_level
from rclpy.executors import MultiThreadedExecutor

import time

# ROS2 messages
from std_msgs.msg import String, Float64
from mavros_msgs.msg import State # For checking armed status
from mavros_msgs.srv import SetMode, CommandBool, CommandTOL, WaypointSetCurrent, CommandLong
from mavros_msgs.msg import StatusText # For status text from FCU

class MissionCommander(Node):
    """
    A ROS2 node to command a MAVROS-enabled vehicle through a mission sequence.
    It subscribes to '/mission_commands' for high-level instructions and
    interacts with MAVROS services for vehicle control (mode, arming, takeoff, mission).
    """

    def __init__(self):
        super().__init__('mission_commander')

        # State variables to track current status (optional but good practice)
        self._current_mode = ""
        self._is_armed = False
        self._takeoff_complete = False

        # Create clients for MAVROS services
        self._set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self._arm_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self._takeoff_client = self.create_client(CommandTOL, '/mavros/cmd/takeoff')
        self._set_waypoint_client = self.create_client(WaypointSetCurrent, '/mavros/mission/set_current')
        # No new client needed for RTL/LAND if using set_mode, or for disarm if using arm_client

        # Ensure service clients are ready before attempting calls
        self._wait_for_all_services()

        # Subscription for mission commands
        self.command_sub = self.create_subscription(String, '/mission_commands', self._command_cb, 1)
        self.get_logger().info("Subscribed to /mission_commands topic. Waiting for commands.")

    def _wait_for_all_services(self):
        """Waits for all necessary MAVROS services to be available."""
        self.get_logger().info("Waiting for MAVROS services...")
        # Wait for each service client to be available
        while not self._set_mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /mavros/set_mode service...')
        while not self._arm_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /mavros/cmd/arming service...')
        while not self._takeoff_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /mavros/cmd/takeoff service...')
        while not self._set_waypoint_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /mavros/mission/set_current service...')
        self.get_logger().info("All MAVROS services are available.")


    def _command_cb(self, msg):
        """
        Callback function for the /mission_commands topic.
        Processes incoming commands and triggers appropriate actions.
        """
        command = msg.data.lower() # Convert to lowercase for case-insensitive matching
        self.get_logger().info(f"Received command: '{command}'")

        if command == "mission_start":
            self.get_logger().info("Received 'mission_start' command. Initiating mission sequence.")
            self.handle_mission_start()
        elif command == "rtl":
            self.get_logger().info("Received 'rtl' command. Returning to Launch.")
            self.return_to_launch()
        elif command == "land":
            self.get_logger().info("Received 'land' command. Initiating landing sequence.")
            self.land()
        elif command == "disarm":
            self.get_logger().info("Received 'disarm' command. Disarming vehicle.")
            self.disarm_vehicle()
        else:
            self.get_logger().warn(f"Unknown command received: '{command}'. Ignoring.")

    def handle_mission_start(self):
        """
        Initiates the mission start sequence:
        1. Sets mode to GUIDED.
        2. Arms the vehicle (upon successful GUIDED mode).
        3. Commands takeoff (upon successful arming).
        4. Sets mode to AUTO and starts mission from waypoint 0 (upon successful takeoff command).
        """
        self.get_logger().info("Attempting to start mission sequence: Requesting GUIDED mode...")
        self.set_mode("GUIDED")

    def set_mode(self, mode_str):
        """
        Sends a request to MAVROS to change the vehicle's flight mode.
        :param mode_str: The desired flight mode (e.g., "GUIDED", "AUTO", "RTL", "LAND").
        """
        self.get_logger().info(f"Requesting mode change to: {mode_str}")
        req = SetMode.Request()
        req.custom_mode = mode_str

        # Make the asynchronous call and attach a callback
        future = self._set_mode_client.call_async(req)
        future.add_done_callback(lambda future: self._set_mode_response_cb(future, mode_str))

    def _set_mode_response_cb(self, future, requested_mode):
        """Callback for set_mode service response."""
        try:
            response = future.result()
            if response is not None and response.mode_sent:
                self._current_mode = requested_mode
                self.get_logger().info(f"✅ Successfully set mode to {requested_mode}")

                # Chain the next action based on the mode set
                if requested_mode == "GUIDED":
                    self.arm(True) # Arm after successfully setting GUIDED mode
                elif requested_mode == "AUTO":
                    # Only set waypoint if takeoff was completed as part of mission_start
                    if self._takeoff_complete:
                        self.set_current_waypoint(0) # Set waypoint after successful AUTO mode transition post-takeoff
                elif requested_mode == "RTL":
                    self.get_logger().info("Vehicle is now in RTL mode. It will return to launch.")
                elif requested_mode == "LAND":
                    self.get_logger().info("Vehicle is now in LAND mode. It will land at current position.")
            else:
                self.get_logger().error(f"❌ Failed to set mode to {requested_mode}. Response: {response}")
        except Exception as e:
            self.get_logger().error(f"Service call failed for set_mode: {e}")

    def arm(self, value=True):
        """
        Sends a request to MAVROS to arm or disarm the vehicle.
        :param value: True to arm, False to disarm.
        """
        action = "arming" if value else "disarming"
        self.get_logger().info(f"Requesting {action}: {value}")
        req = CommandBool.Request()
        req.value = value

        # Make the asynchronous call and attach a callback
        future = self._arm_client.call_async(req)
        future.add_done_callback(lambda future: self._arm_response_cb(future, value))

    def _arm_response_cb(self, future, requested_value):
        """Callback for arm service response."""
        try:
            response = future.result()
            if response is not None and response.success:
                self._is_armed = requested_value
                if requested_value:
                    self.get_logger().info("✅ Vehicle armed successfully.")
                    self.takeoff(altitude=10.0) # Takeoff after successfully arming
                else:
                    self.get_logger().info("✅ Vehicle disarmed successfully.")
            else:
                action = "arm" if requested_value else "disarm"
                self.get_logger().error(f"❌ Failed to {action} vehicle. Response: {response}")
        except Exception as e:
            self.get_logger().error(f"Service call failed for arm/disarm: {e}")

    def takeoff(self, altitude=10.0):
        """
        Sends a takeoff command to MAVROS.
        :param altitude: The target altitude for takeoff in meters.
        """
        self.get_logger().info(f"Requesting takeoff to {altitude}m.")
        req = CommandTOL.Request()
        req.min_pitch = 0.0
        req.yaw = 0.0
        req.latitude = 0.0  # Use 0 if GPS not needed, or actual coordinates if available
        req.longitude = 0.0
        req.altitude = altitude

        # Make the asynchronous call and attach a callback
        future = self._takeoff_client.call_async(req)
        future.add_done_callback(lambda future: self._takeoff_response_cb(future, altitude))

    def _takeoff_response_cb(self, future, requested_altitude):
        """Callback for takeoff service response."""
        try:
            response = future.result()
            if response is not None and response.success:
                self._takeoff_complete = True
                self.get_logger().info(f"✅ Takeoff command sent, target altitude: {requested_altitude}m")
                self.set_mode("AUTO") # Set to AUTO mode after takeoff command is sent
            else:
                self.get_logger().error(f"❌ Takeoff failed. Response: {response}")
        except Exception as e:
            self.get_logger().error(f"Service call failed for takeoff: {e}")

    def set_current_waypoint(self, wp_seq):
        """
        Sets the current waypoint in an AUTO mission.
        :param wp_seq: The sequence number of the waypoint to set as current.
        """
        self.get_logger().info(f"Setting current waypoint to {wp_seq}.")
        req = WaypointSetCurrent.Request()
        req.wp_seq = wp_seq

        # Make the asynchronous call and attach a callback
        future = self._set_waypoint_client.call_async(req)
        future.add_done_callback(self._set_waypoint_response_cb)

    def _set_waypoint_response_cb(self, future):
        """Callback for set_current_waypoint service response."""
        try:
            response = future.result()
            if response is not None and response.success:
                self.get_logger().info("✅ Mission started from waypoint 0.")
            else:
                self.get_logger().error(f"❌ Failed to set current waypoint. Response: {response}")
        except Exception as e:
            self.get_logger().error(f"Service call failed for set_current_waypoint: {e}")

    def return_to_launch(self):
        """Commands the vehicle to return to its launch position (RTL mode)."""
        self.get_logger().info("Commanding vehicle to Return To Launch (RTL).")
        self.set_mode("RTL")

    def land(self):
        """Commands the vehicle to land at its current position (LAND mode)."""
        self.get_logger().info("Commanding vehicle to Land.")
        self.set_mode("LAND")

    def disarm_vehicle(self):
        """Commands the vehicle to disarm."""
        self.get_logger().info("Commanding vehicle to Disarm.")
        self.arm(False)

def main(args=None):
    """
    Main function to initialize and run the ROS2 node.
    """
    rclpy.init(args=args)
    node = MissionCommander()

    # Create a MultiThreadedExecutor to handle callbacks concurrently
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        # Spin the executor to process all callbacks (subscriptions and service responses)
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Mission Commander node.')
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
