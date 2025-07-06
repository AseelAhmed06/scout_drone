import rclpy
from rclpy.node import Node
from mavros_msgs.srv import SetMode, CommandBool
# Import other service types as needed:
# from mavros_msgs.srv import WaypointPush # for waypoints

class DroneControlNode(Node):
    def __init__(self):
        super().__init__('drone_control_node')

        # Create service clients
        self.arm_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        # self.waypoint_push_client = self.create_client(WaypointPush, '/mavros/mission/push') # Add this for waypoint services

        # Wait for services to be available (good practice)
        self.get_logger().info("Waiting for arming service...")
        while not self.arm_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Arming service not available, waiting again...')
        self.get_logger().info("Arming service available.")

        self.get_logger().info("Waiting for set_mode service...")
        while not self.set_mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('SetMode service not available, waiting again...')
        self.get_logger().info("SetMode service available.")

    def send_arming_command(self, arm: bool) -> bool:
        """Sends an arm/disarm command to the FCU and waits for the response."""
        request = CommandBool.Request()
        request.value = arm
        self.get_logger().info(f'Sending arming request: {arm}')
        future = self.arm_client.call_async(request)

        # THIS IS THE CRUCIAL PART: Spin the node until the future is complete
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            if future.result().success:
                self.get_logger().info(f"Arming {'successful' if arm else 'disarm successful'}. Result: {future.result().result}")
                return True
            else:
                self.get_logger().warn(f"Arming {'failed' if arm else 'disarm failed'}. Result code: {future.result().result}")
                return False
        else:
            self.get_logger().error("Arming service call failed (no response or exception).")
            return False

    def send_set_mode_command(self, custom_mode: str) -> bool:
        """Sends a set_mode command to the FCU and waits for the response."""
        request = SetMode.Request()
        request.custom_mode = custom_mode
        # In PX4, you might also need request.base_mode = 0 for custom_mode only
        # In ArduPilot, custom_mode is often enough

        self.get_logger().info(f'Sending set mode request: {custom_mode}')
        future = self.set_mode_client.call_async(request)

        # CRUCIAL: Spin until the future is complete
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            if future.result().mode_sent:
                self.get_logger().info(f"Mode change to {custom_mode} successful.")
                return True
            else:
                self.get_logger().warn(f"Mode change to {custom_mode} failed. Mode not sent.")
                return False
        else:
            self.get_logger().error("SetMode service call failed (no response or exception).")
            return False

    # Add similar methods for other service calls (WaypointPush, etc.)

def main(args=None):
    rclpy.init(args=args)
    node = DroneControlNode()

    # --- Example Usage ---
    try:
        # Example 1: Change mode to GUIDED
        if node.send_set_mode_command("GUIDED"): # Or "OFFBOARD" for PX4
            node.get_logger().info("Successfully set mode to GUIDED.")
        else:
            node.get_logger().error("Failed to set mode.")

        # Example 2: Arm the drone
        # IMPORTANT: For PX4 OFFBOARD, you MUST be streaming setpoints before arming
        # For ArduPilot GUIDED_NOGPS/GUIDED, you should also stream.
        # This example assumes you have another part of your code streaming setpoints.
        # For a quick test, you might need to manually ensure setpoints are published.
        if node.send_arming_command(True):
            node.get_logger().info("Successfully armed drone!")
            # Add some delay or flight logic here
            # node.get_logger().info("Disarming in 5 seconds...")
            # time.sleep(5)
            # node.send_arming_command(False)
        else:
            node.get_logger().error("Failed to arm drone.")

    except Exception as e:
        node.get_logger().error(f"An error occurred: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()