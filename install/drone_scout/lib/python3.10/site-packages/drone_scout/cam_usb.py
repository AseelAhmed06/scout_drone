import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32
import subprocess
import os
import time
import threading
from datetime import datetime
from mavros_msgs.msg import WaypointReached
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

class ImageCaptureNode(Node):
    """
    A ROS2 node to capture images from a USB camera using ffmpeg.
    Image capture starts and stops based on received waypoint numbers.
    Captured image filenames and their timestamps are published to a topic.
    """

    def __init__(self):
        super().__init__('image_capture_node')

        # Declare ROS2 parameters for configuration
        self.declare_parameter('folder_path', '/tmp/camera_images')
        self.declare_parameter('camera_device', '/dev/video0')
        self.declare_parameter('fps', 1)
        self.declare_parameter('video_size', '1920x1080')
        self.declare_parameter('start_waypoint_index', 5)
        self.declare_parameter('stop_waypoint_index', -1)
        self.declare_parameter('image_prefix', 'nvcamtest_')
        self.declare_parameter('image_extension', '.jpg')

        # Get parameter values
        self.image_save_directory = self.get_parameter('folder_path').get_parameter_value().string_value
        self.camera_device = self.get_parameter('camera_device').get_parameter_value().string_value
        self.frame_rate = int(self.get_parameter('fps').value)
        self.video_size = self.get_parameter('video_size').get_parameter_value().string_value
        self.start_waypoint = self.get_parameter('start_waypoint_index').value
        self.stop_waypoint = self.get_parameter('stop_waypoint').value
        self.image_prefix = self.get_parameter('image_prefix').get_parameter_value().string_value
        self.image_extension = self.get_parameter('image_extension').get_parameter_value().string_value
        self.image_save_directory = os.path.join(self.image_save_directory, "captured_frames")
        # Create the image save directory if it does not exist
        os.makedirs(self.image_save_directory, exist_ok=True)
        self.get_logger().info(f"Image save directory set to: {self.image_save_directory}")

        # Internal state variables
        self._is_capturing = False  # Flag to indicate if image capture is active
        self._ffmpeg_process = None # Stores the subprocess.Popen object for ffmpeg
        self._last_published_filename = "" # Stores the name of the last image file published
        self._ffmpeg_stderr_thread = None # Thread to consume ffmpeg's stderr output
        qos_profile_system_default = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        # ROS2 Publishers and Subscribers
        # Publisher for image timestamps and filenames
        self.timestamp_publisher = self.create_publisher(String, "/camera/timestamps", 10)
        # Subscriber for waypoint messages
        self.create_subscription(WaypointReached, '/mavros/mission/reached', self.waypoint_callback, 10)
        self.get_logger().info("ImageCaptureNode initialized and ready.")
        self.create_subscription(
            String,
            '/drone_status/last_waypoint_index',
            self.waypoint_index_callback,
            qos_profile_system_default
        )
        # Timer for periodically checking for new images.
        # This timer will be created and destroyed dynamically when capture starts/stops.
        self._check_image_timer = None

    def waypoint_index_callback(self, msg: String):
        """
        Callback function for the /drone_status/last_waypoint_index topic.
        Converts the received string data to an integer.
        """
        try:
           
            last_waypoint_index_int = int(msg.data)
            self.get_logger().info(f'Received last waypoint index (int): {last_waypoint_index_int}')
            self.stop_waypoint = last_waypoint_index_int
        except ValueError:
            self.get_logger().error(f'Failed to convert received data "{msg.data}" to integer. Is the publisher sending valid integer strings?')
        except Exception as e:
            self.get_logger().error(f'An unexpected error occurred: {e}')


    def waypoint_callback(self, msg):
        """
        Callback function for the /waypoint topic.
        Controls starting and stopping of image capture based on the received waypoint.
        """
        current_waypoint = msg.wp_seq
        self.get_logger().info(f"Received waypoint: {current_waypoint}")

        if current_waypoint == self.start_waypoint and not self._is_capturing:
            self.get_logger().info(f"Waypoint {current_waypoint} reached. Starting image capture.")
            self._start_image_capture()
        elif current_waypoint == self.stop_waypoint and self._is_capturing:
            self.get_logger().info(f"Waypoint {current_waypoint} reached. Stopping image capture.")
            self._stop_image_capture()

    def _start_image_capture(self):
        """
        Starts the ffmpeg process to capture images.
        """
        # Construct the output filename pattern for ffmpeg.
        # %Y%m%d_%H%M%S_%f ensures unique, chronologically sortable filenames
        # by including year, month, day, hour, minute, second, and microseconds.
        output_filename_pattern = os.path.join(
            self.image_save_directory,
            f"{self.image_prefix}%Y%m%d_%H%M%S_%f{self.image_extension}"
        )
        
        # FFmpeg command to capture from V4L2 device, set framerate, video size,
        # apply a video filter to ensure exactly 1 fps output, and set JPEG quality.
        ffmpeg_command = [
            'ffmpeg',
            '-y', # Overwrite output files without asking
            '-f', 'v4l2', # Input format is Video4Linux2
            '-framerate', str(self.frame_rate), # Input framerate
            '-video_size', self.video_size, # Input video resolution
            '-i', self.camera_device, # Input camera device
            '-vf', f'fps={self.frame_rate}', # Video filter to enforce output framerate
            '-q:v', '2', # JPEG quality (2 is generally good)
            output_filename_pattern # Output file pattern
        ]

        self.get_logger().info(f"Executing FFmpeg command: {' '.join(ffmpeg_command)}")

        try:
            # Start the ffmpeg process.
            # stdout and stderr are piped to allow reading their output.
            # universal_newlines=True decodes output as text.
            self._ffmpeg_process = subprocess.Popen(
                ffmpeg_command,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                universal_newlines=True
            )
            self._is_capturing = True
            self._last_published_filename = "" # Reset the last published filename when starting capture

            # Create and start a timer to periodically check for new images.
            # The timer frequency matches the capture framerate to ensure timely detection.
            self._check_image_timer = self.create_timer(1.0 / self.frame_rate, self._check_for_new_images)

            # Start a separate thread to continuously read ffmpeg's stderr.
            # This prevents the stderr buffer from filling up and blocking the ffmpeg process.
            self._ffmpeg_stderr_thread = threading.Thread(target=self._read_ffmpeg_stderr, args=(self._ffmpeg_process,))
            self._ffmpeg_stderr_thread.daemon = True # Allow main program to exit even if thread is running
            self._ffmpeg_stderr_thread.start()

            self.get_logger().info("FFmpeg process successfully started.")

        except FileNotFoundError:
            self.get_logger().error("FFmpeg command not found. Please ensure ffmpeg is installed and in your system's PATH.")
            self._is_capturing = False
            self._ffmpeg_process = None
        except Exception as e:
            self.get_logger().error(f"Failed to start ffmpeg process: {e}")
            self._is_capturing = False
            self._ffmpeg_process = None

    def _read_ffmpeg_stderr(self, process):
        """
        Reads stderr from the ffmpeg process.
        This function runs in a separate thread to prevent stderr buffer overflow.
        Output is consumed but not necessarily logged unless debugging.
        """
        while self._is_capturing and process.poll() is None:
            line = process.stderr.readline()
            if line:
                # self.get_logger().debug(f"FFmpeg stderr: {line.strip()}")
                pass # Consume the output to prevent blocking

    def _stop_image_capture(self):
        """
        Stops the ffmpeg process and cleans up resources.
        """
        if self._ffmpeg_process:
            self.get_logger().info("Terminating FFmpeg process...")
            self._ffmpeg_process.terminate() # Send SIGTERM (graceful termination)
            try:
                self._ffmpeg_process.wait(timeout=5) # Wait for the process to terminate
                self.get_logger().info("FFmpeg process terminated gracefully.")
            except subprocess.TimeoutExpired:
                self.get_logger().warn("FFmpeg process did not terminate gracefully within 5 seconds, killing it.")
                self._ffmpeg_process.kill() # Force kill if it doesn't respond to SIGTERM
                self._ffmpeg_process.wait() # Wait for the kill to complete
            self._ffmpeg_process = None
        
        # Cancel the image checking timer if it's active
        if self._check_image_timer:
            self._check_image_timer.cancel()
            self._check_image_timer = None

        self._is_capturing = False
        self.get_logger().info("Image capture stopped.")

    def _check_for_new_images(self):
        """
        Periodically checks the image save directory for new image files
        and publishes their details.
        This function is called by the self._check_image_timer.
        """
        if not self._is_capturing:
            return

        try:
            # List all files in the designated directory
            all_files = os.listdir(self.image_save_directory)

            # Filter for files that match the expected prefix and extension
            image_files = [
                f for f in all_files
                if f.startswith(self.image_prefix) and f.endswith(self.image_extension)
            ]

            if not image_files:
                # self.get_logger().debug("No image files found yet matching pattern in the directory.")
                return

            # Sort the files. Since filenames include timestamps, sorting them
            # alphabetically will also sort them chronologically.
            image_files.sort()

            # The newest image will be the last one in the sorted list
            newest_image_filename = image_files[-1]

            if newest_image_filename == self._last_published_filename:
                # This image has already been published, so do nothing.
                # self.get_logger().debug(f"Image {newest_image_filename} already published.")
                return 

            # Construct the full path to the newest image
            full_image_path = os.path.join(self.image_save_directory, newest_image_filename)

            # Get the modification timestamp of the file.
            # os.path.getmtime returns time in seconds since the epoch as a float.
            # Convert it to nanoseconds as requested.
            timestamp_s = os.path.getmtime(full_image_path)
            timestamp_ns = int(timestamp_s * 1_000_000_000)

            # Create and publish the String message
            msg = String()
            msg.data = f"{timestamp_ns},{newest_image_filename}"
            self.timestamp_publisher.publish(msg)
            self.get_logger().info(f"Published image data: {msg.data}")

            # Update the last published filename to avoid re-publishing the same image
            self._last_published_filename = newest_image_filename

        except Exception as e:
            self.get_logger().error(f"Error checking for new images: {e}")

    def destroy_node(self):
        """
        Called when the node is being destroyed.
        Ensures the ffmpeg process is stopped cleanly.
        """
        self._stop_image_capture() # Stop ffmpeg if it's running
        super().destroy_node() # Call the base class's destroy_node method

def main(args=None):
    """
    Main function to initialize and run the ROS2 node.
    """
    rclpy.init(args=args) # Initialize ROS2 client library
    node = ImageCaptureNode() # Create an instance of the node
    try:
        rclpy.spin(node) # Keep the node alive and processing callbacks
    except KeyboardInterrupt:
        node.get_logger().info("Node interrupted by user.")
    finally:
        node.destroy_node() # Clean up node resources
        rclpy.shutdown() # Shut down ROS2 client library

if __name__ == '__main__':
    main()
