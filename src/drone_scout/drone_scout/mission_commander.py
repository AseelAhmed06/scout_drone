import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy, qos_profile_sensor_data

import time

# ROS2 messages
from std_msgs.msg import String, Float64
from mavros_msgs.msg import State # For checking armed status
from mavros_msgs.srv import SetMode, CommandBool, CommandTOL, WaypointSetCurrent

class MissionCommander(Node):
    """
    A ROS2 node that subscribes to a command topic and triggers mission
    start procedures based on received commands. It handles arming,
    takeoff, and setting the vehicle to AUTO mode.
    """

    def __init__(self):
        super().__init__('mission_commander')
        self.get_logger().info("Mission Commander Node Initialized.")

        # QoS profile for reliable communication.
        # This is important for critical MAVROS topics to ensure messages are not lost.
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # --- State Variables ---
        # Stores the current state of the MAV (e.g., armed, connected, mode).
        self.current_state = State()
        # Stores the current relative altitude of the MAV in meters.
        self.current_rel_alt = 0.0 

        # --- Parameters ---
        # Declare a parameter for the desired takeoff altitude.
        # This allows users to configure the altitude without modifying code.
        self.declare_parameter('takeoff_altitude', 5.0) # Default takeoff altitude in meters
        self.takeoff_altitude = self.get_parameter('takeoff_altitude').value
        self.get_logger().info(f"Configured takeoff altitude: {self.takeoff_altitude}m")

        # --- Subscribers ---
        # Subscribe to MAVROS state topic to get arming status and other vehicle states.
        self.state_sub = self.create_subscription(
            State,
            '/mavros/state',
            self._state_cb,
            qos_profile
        )
        self.get_logger().info("Subscribed to /mavros/state topic.")

        # Subscribe to relative altitude topic to know the vehicle's height above home.
        self.rel_alt_sub = self.create_subscription(
            Float64,
            '/mavros/global_position/rel_alt',
            self._rel_alt_cb,
            qos_profile_sensor_data
        )
        self.get_logger().info("Subscribed to /mavros/global_position/rel_alt topic.")

        # Subscribe to a custom mission commands topic.
        # This topic will receive commands like "mission_start".
        self.command_sub = self.create_subscription(
            String,
            '/mission_commands', # Topic to listen for commands
            self._command_cb,
            qos_profile
        )
        self.get_logger().info("Subscribed to /mission_commands topic. Waiting for 'mission_start' command.")

        # --- Service Clients ---
        # Create clients for MAVROS services required for controlling the vehicle.
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.takeoff_client = self.create_client(CommandTOL, '/mavros/cmd/takeoff')
        self.set_current_wp_client = self.create_client(WaypointSetCurrent, '/mavros/mission/set_current')

        # Wait for all necessary MAVROS services to be available before proceeding.
        # This ensures that service calls don't fail due to the service not being up yet.
        self._wait_for_service(self.set_mode_client, '/mavros/set_mode')
        self._wait_for_service(self.arming_client, '/mavros/cmd/arming')
        self._wait_for_service(self.takeoff_client, '/mavros/cmd/takeoff')
        self._wait_for_service(self.set_current_wp_client, '/mavros/mission/set_current')

    def _wait_for_service(self, client, service_name):
        """
        Helper method to block and wait until a specified ROS service is available.
        """
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'Waiting for {service_name} service...')
        self.get_logger().info(f'{service_name} service is available.')

    def _state_cb(self, msg):
        """
        Callback function for the /mavros/state topic.
        Updates the node's internal current_state variable.
        """
        self.current_state = msg

    def _rel_alt_cb(self, msg):
        """
        Callback function for the /mavros/global_position/rel_alt topic.
        Updates the node's internal current_rel_alt variable.
        """
        self.current_rel_alt = msg.data

    def _command_cb(self, msg):
        """
        Callback function for the /mission_commands topic.
        Processes incoming commands and triggers appropriate actions.
        """
        command = msg.data
        self.get_logger().info(f"Received command: '{command}'")

        if command == "mission_start":
            self.handle_mission_start()
        else:
            self.get_logger().warn(f"Unknown command received: '{command}'. Ignoring.")

    def handle_mission_start(self):
        """
        Handles the 'mission_start' command.
        Checks vehicle state (armed, altitude) and performs necessary actions:
        - If already armed and at altitude: sets mode to AUTO and starts mission.
        - Otherwise: attempts to arm, takeoff, then sets mode to AUTO and starts mission.
        """
        self.get_logger().info("Attempting to start mission...")

        # Check if the vehicle is currently armed and has reached a sufficient relative altitude.
        # A threshold of 1.0m is used to consider it "at altitude".
        if self.current_state.armed and self.current_rel_alt > 1.0:
            self.get_logger().info("✅ Vehicle armed and at sufficient altitude. Proceeding to AUTO mode.")
            # Set the flight mode to "AUTO" (for AUTO.MISSION).
            self._set_mode("AUTO")
            time.sleep(1) # Give a short delay for the mode change to propagate in the FCU.
            # Set the current waypoint to 0 to start the mission from the beginning.
            self._set_current_waypoint(0)
        else:
            self.get_logger().info("❌ Vehicle not armed or not at sufficient altitude. Performing arm and takeoff.")
            
            # 1. Arm the vehicle if it's not already armed.
            if not self.current_state.armed:
                self.get_logger().info("Vehicle is not armed. Attempting to arm...")
                if not self._arm(True): # Call the arming service
                    self.get_logger().error("Failed to arm vehicle. Aborting mission start.")
                    return # Exit if arming fails

                # Wait for a moment for the arming state to update.
                time.sleep(2) 
                # Re-check arming status after attempting to arm.
                if not self.current_state.armed:
                    self.get_logger().error("Vehicle did not report armed after arming command. Aborting mission start.")
                    return

            # 2. Command a takeoff to the configured altitude.
            self.get_logger().info(f"Vehicle armed. Attempting takeoff to {self.takeoff_altitude}m...")
            if not self._takeoff(self.takeoff_altitude): # Call the takeoff service
                self.get_logger().error("Takeoff command failed. Aborting mission start.")
                return

            # Wait for the vehicle to reach the takeoff altitude.
            # This sleep duration might need adjustment based on vehicle performance.
            time.sleep(5) 

            # Re-check altitude after takeoff attempt to provide feedback.
            if self.current_rel_alt < (self.takeoff_altitude * 0.8): # Check if close to target altitude (e.g., 80% of target)
                self.get_logger().warn(f"Vehicle might not have reached target takeoff altitude ({self.takeoff_altitude}m). Current rel_alt: {self.current_rel_alt}m")
            else:
                self.get_logger().info(f"Vehicle appears to be at target altitude: {self.current_rel_alt}m")

            # 3. Set flight mode to "AUTO" (for AUTO.MISSION).
            self.get_logger().info("Setting mode to AUTO to begin mission execution.")
            self._set_mode("AUTO")
            time.sleep(1) # Give a short delay for the mode change to propagate.
            
            # 4. Set current waypoint to 0 to start the mission from the beginning.
            self.get_logger().info("Setting current waypoint to 0 to initiate mission.")
            self._set_current_waypoint(0)

    def _set_mode(self, mode_str):
        """
        Helper method to call the MAVROS set_mode service.
        Returns True on success, False otherwise.
        """
        req = SetMode.Request()
        req.custom_mode = mode_str
        
        self.get_logger().info(f"Requesting mode change to: {mode_str}")
        future = self.set_mode_client.call_async(req)
        # Block until the service call completes.
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None and future.result().mode_sent:
            self.get_logger().info(f"✅ Successfully set mode to {mode_str}.")
            return True
        else:
            self.get_logger().error(f"❌ Failed to set mode to {mode_str}. Result: {future.result()}")
            return False

    def _arm(self, value=True):
        """
        Helper method to call the MAVROS arming service.
        Returns True on success, False otherwise.
        """
        req = CommandBool.Request()
        req.value = value
        
        action = "Arming" if value else "Disarming"
        self.get_logger().info(f"Requesting vehicle {action}.")
        future = self.arming_client.call_async(req)
        # Block until the service call completes.
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None and future.result().success:
            self.get_logger().info(f"✅ Vehicle {action.lower()} successful.")
            return True
        else:
            self.get_logger().error(f"❌ Failed to {action.lower()} vehicle. Result: {future.result()}")
            return False

    def _takeoff(self, altitude):
        """
        Helper method to call the MAVROS takeoff service.
        Commands the vehicle to takeoff to a specified altitude.
        Returns True on success, False otherwise.
        """
        req = CommandTOL.Request()
        req.min_pitch = 0.0 # Minimum pitch for takeoff (usually 0 for vertical takeoff)
        req.yaw = 0.0       # Desired yaw angle after takeoff (0 means current yaw)
        req.latitude = 0.0  # Use 0 if GPS coordinates are not needed for the takeoff point
        req.longitude = 0.0 # Use 0 if GPS coordinates are not needed for the takeoff point
        req.altitude = altitude # Target altitude for takeoff

        self.get_logger().info(f"Requesting takeoff to altitude: {altitude}m")
        future = self.takeoff_client.call_async(req)
        # Block until the service call completes.
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None and future.result().success:
            self.get_logger().info(f"✅ Takeoff command sent successfully to {altitude}m.")
            return True
        else:
            self.get_logger().error(f"❌ Takeoff command failed. Result: {future.result()}")
            return False

    def _set_current_waypoint(self, wp_seq):
        """
        Helper method to call the MAVROS set_current waypoint service.
        Sets the mission's current waypoint index.
        Returns True on success, False otherwise.
        """
        req = WaypointSetCurrent.Request()
        req.wp_seq = wp_seq # The index of the waypoint to set as current
        
        self.get_logger().info(f"Setting current waypoint to index: {wp_seq}")
        future = self.set_current_wp_client.call_async(req)
        # Block until the service call completes.
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None and future.result().success:
            self.get_logger().info(f"✅ Mission started from waypoint {wp_seq}.")
            return True
        else:
            self.get_logger().error(f"❌ Failed to set current waypoint to {wp_seq}. Result: {future.result()}")
            return False

def main(args=None):
    """
    Main function to initialize and run the ROS2 node.
    """
    rclpy.init(args=args)
    node = MissionCommander()
    # Keep the node alive to continuously receive commands and process callbacks.
    rclpy.spin(node) 
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

