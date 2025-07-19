#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib
import sys
import csv
import time
import datetime
import os
import threading
import subprocess
import signal
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

# Import OpenCV and NumPy for image processing and saving
import cv2
import numpy as np

# ROS2 Imports
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Int32 # Using Int32 for waypoint messages


# --- GStreamer Globals ---
# These remain global as they are shared between the main thread (Node) and the GStreamer thread/callbacks.
# They will be initialized from parameters within the Node class.
first_argus_frame_system_time_ns = None
first_argus_frame_capture_time_ns = None
time_offset_ns = 0

frame_count = 0
pipeline = None
bus = None
gst_loop = None

# CSV file handle (will be managed by the Node instance)
csv_file = None
csv_writer = None

class CameraCaptureNode(Node):
    """
    ROS2 Node for capturing camera frames using GStreamer,
    timestamping them, saving images, logging to CSV,
    and controlling capture based on MAVROS waypoints.
    """
    def __init__(self):
        super().__init__('camera_capture_node')

        # Initialize Publishers early, so status messages can be sent immediately.
        self.timestamp_publisher = self.create_publisher(String, '/camera/timestamps', 5)
        self.status_publisher = self.create_publisher(String, '/camera/status', 5)

        # Waypoint control variables
        self.is_capturing = False # Flag to control if camera is actively capturing
        self.current_waypoint_index = -1 # To store the latest waypoint received
        qos_profile_system_default = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        # Declare and get all parameters
        self.declare_parameter('start_waypoint_index', -1)
        self.declare_parameter('stop_waypoint_index', -1)
        self.declare_parameter('csv_filename', "/home/edhitha/python_absolute_timestamps_ros.csv")
        self.declare_parameter('fps', 1.0)
        self.declare_parameter('folder_path', "/home/edhitha/captured_frames")

        self.start_waypoint_index = self.get_parameter('start_waypoint_index').value
        self.stop_waypoint_index = self.get_parameter('stop_waypoint_index').value
        self.csv_filename = self.get_parameter('csv_filename').get_parameter_value().string_value
        self.fps = self.get_parameter('fps').value
        self.frame_save_dir = self.get_parameter('folder_path').get_parameter_value().string_value
        self.frame_save_dir = os.path.join(self.frame_save_dir, "captured_frames")

        self.get_logger().info(
            f"INFO: Successfully loaded parameters:\n"
            f"  START_WAYPOINT={self.start_waypoint_index}\n"
            f"  STOP_WAYPOINT={self.stop_waypoint_index}\n"
            f"  CSV_FILENAME={self.csv_filename}\n"
            f"  FPS={self.fps}\n"
            f"  FRAME_SAVE_DIR={self.frame_save_dir}"
        )

        # Subscribe to MAVROS current waypoint topic.
        self.create_subscription(
            Int32, # Message type: std_msgs/Int32
            '/mavros/mission/reached', # Topic name
            self.waypoint_callback, # Callback function
            10 # Queue size
        )
        self.create_subscription(
            String,
            '/drone_status/last_waypoint_index',
            self.waypoint_index_callback,
            qos_profile_system_default
        )
        self.get_logger().info("INFO: Subscribed to /mavros/mission/reached for waypoint control.")
        self.get_logger().info("INFO: Press Ctrl+C to stop this node.")

        # Start the GStreamer pipeline in a separate thread.
        self.gst_thread = threading.Thread(target=self.gst_thread_main)
        self.gst_thread.daemon = True # Allow the main program to exit even if this thread is running
        self.gst_thread.start()
        self.get_logger().info("INFO: GStreamer initialization thread started.")

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


    def publish_status(self, message: str):
        """
        Helper function to publish status messages to the /camera/status topic.
        """
        status_msg = String()
        status_msg.data = message
        self.status_publisher.publish(status_msg)
        self.get_logger().info(f"[STATUS_PUB] {message}")

    def start_capture(self):
        """
        Sets the GStreamer pipeline to PLAYING state to start camera capture.
        Resets synchronization offset for a fresh capture session.
        """
        global pipeline, first_argus_frame_system_time_ns, first_argus_frame_capture_time_ns, time_offset_ns
        if not self.is_capturing:
            if pipeline is None:
                self.publish_status("ERROR: Cannot start capture, GStreamer pipeline is not initialized yet.")
                return

            self.publish_status("Attempting to START camera capture...")

            # Reset offset for fresh sync when starting capture again
            first_argus_frame_system_time_ns = None
            first_argus_frame_capture_time_ns = None
            time_offset_ns = 0

            # Set pipeline to PLAYING state
            ret = pipeline.set_state(Gst.StateChangeReturn.ASYNC) # Use ASYNC for state changes
            if ret == Gst.StateChangeReturn.FAILURE:
                self.publish_status("ERROR: Failed to set pipeline to PLAYING state. Check GStreamer errors.")
                return
            self.is_capturing = True
            self.publish_status("Camera capture **STARTED**.")
        else:
            self.publish_status("Camera is already capturing.")

    def stop_capture(self):
        """
        Sets the GStreamer pipeline to NULL state to stop camera capture.
        """
        global pipeline
        if self.is_capturing:
            if pipeline is None:
                self.publish_status("WARNING: Cannot stop capture, GStreamer pipeline is not initialized, "
                                    "but capture flag is set. Correcting flag.")
                self.is_capturing = False # Correct the flag in case of inconsistency
                return

            self.publish_status("Attempting to STOP camera capture...")
            # Set pipeline to NULL state (releases camera resources)
            ret = pipeline.set_state(Gst.StateChangeReturn.ASYNC) # Use ASYNC for state changes
            if ret == Gst.StateChangeReturn.FAILURE:
                self.publish_status("ERROR: Failed to set pipeline to NULL state. Check GStreamer errors.")
                return
            self.is_capturing = False
            self.publish_status("Camera capture **STOPPED**.")
        else:
            self.publish_status("Camera is not currently capturing.")

    def waypoint_callback(self, msg: Int32):
        """
        Callback function for the /mavros/mission/reached topic.
        Controls camera capture based on configured start and stop waypoints.
        """
        new_waypoint_index = msg.data
        if new_waypoint_index != self.current_waypoint_index: # Only log/act on waypoint change
            old_waypoint_index = self.current_waypoint_index
            self.current_waypoint_index = new_waypoint_index
            self.publish_status(f"Waypoint changed from {old_waypoint_index} to {self.current_waypoint_index}.")

            if self.current_waypoint_index == self.start_waypoint_index:
                self.publish_status(f"Detected **START WAYPOINT** ({self.start_waypoint_index}). Triggering capture start.")
                self.start_capture()
            elif self.current_waypoint_index == self.stop_waypoint_index:
                self.publish_status(f"Detected **STOP WAYPOINT** ({self.stop_waypoint_index}). Triggering capture stop.")
                self.stop_capture()
            else:
                self.publish_status(
                    f"Currently at waypoint {self.current_waypoint_index}. "
                    f"Waiting for start ({self.start_waypoint_index}) or stop ({self.stop_waypoint_index}) waypoint."
                )

    def on_new_sample(self, sink):
        """
        Callback function for GStreamer 'new-sample' signal from appsink.
        Processes each captured frame: calculates absolute timestamp, saves image,
        and publishes timestamp/filename to ROS.
        """
        global frame_count, first_argus_frame_system_time_ns, \
               first_argus_frame_capture_time_ns, time_offset_ns, csv_writer

        if not self.is_capturing:
            return Gst.FlowReturn.OK # Do nothing if not actively capturing

        sample = sink.emit("pull-sample")
        if sample:
            buffer = sample.get_buffer()
            argus_capture_time_ns = buffer.pts # Hardware timestamp from Argus camera in nanoseconds

            # If this is the first frame *since capture started*, establish the synchronization offset
            if first_argus_frame_system_time_ns is None:
                first_argus_frame_system_time_ns = int(time.time() * 1000000000) # Current system time in ns
                first_argus_frame_capture_time_ns = argus_capture_time_ns
                time_offset_ns = first_argus_frame_system_time_ns - first_argus_frame_capture_time_ns
                self.publish_status("Synchronization established (new capture session).")
                self.publish_status(f"  System time (ns) at first frame: {first_argus_frame_system_time_ns}")
                self.publish_status(f"  Argus time (ns) of first frame: {first_argus_frame_capture_time_ns}")
                self.publish_status(f"  Calculated offset (ns): {time_offset_ns}")

            # Calculate absolute epoch timestamp for the frame
            absolute_timestamp_ns = argus_capture_time_ns + time_offset_ns

            # Format as local time string for CSV/logging
            local_time_s = float(absolute_timestamp_ns) / 1000000000
            human_readable_time_str = (
                f"{time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(local_time_s))}."
                f"{int(absolute_timestamp_ns % 1000000000 / 1000):06d}"
            )

            frame_count += 1

            # --- Image Saving Logic ---
            caps = sample.get_caps()
            s = caps.get_structure(0)
            width = s.get_value("width")
            height = s.get_value("height")

            success, mapinfo = buffer.map(Gst.MapFlags.READ)
            if not success:
                self.publish_status(f"ERROR: Could not map buffer for frame {frame_count}.")
                buffer.unmap(mapinfo)
                return Gst.FlowReturn.ERROR

            if mapinfo.size == 0:
                self.publish_status(f"WARNING: Received empty buffer for frame {frame_count}. Skipping save.")
                buffer.unmap(mapinfo)
                return Gst.FlowReturn.OK

            try:
                image_data = np.ndarray(
                    (height, width, 4),
                    buffer=mapinfo.data,
                    dtype=np.uint8
                )

                bgr_image = cv2.cvtColor(image_data, cv2.COLOR_RGBA2BGR)

                # Use the parameter-configured FRAME_SAVE_DIR
                image_filename_local = os.path.join(self.frame_save_dir, f"frame_{frame_count:06d}.jpg")

                if not cv2.imwrite(image_filename_local, bgr_image):
                    self.publish_status(f"ERROR: Failed to save image file: {image_filename_local}")
                else:
                    self.publish_status(f"Saved frame {frame_count} to: {image_filename_local}")
            except Exception as e:
                self.publish_status(f"ERROR: Image processing/saving failed for frame {frame_count}: {e}")
            finally:
                buffer.unmap(mapinfo)

            # --- ROS Publishing Logic (Timestamp and Filename as String) ---
            message = f"{absolute_timestamp_ns},{os.path.basename(image_filename_local)}"

            if self.timestamp_publisher:
                self.timestamp_publisher.publish(String(data=message))
            else:
                self.publish_status("WARNING: Timestamp publisher not initialized! Cannot publish timestamps.")
            # --- End ROS Publishing Logic ---

            # Log to CSV file
            try:
                csv_writer.writerow([frame_count, argus_capture_time_ns, absolute_timestamp_ns, human_readable_time_str, os.path.basename(image_filename_local)])
                if csv_file:
                    csv_file.flush()
            except Exception as e:
                self.publish_status(f"ERROR: Writing to CSV failed for frame {frame_count}: {e}")

        return Gst.FlowReturn.OK

    def on_message(self, bus, message):
        """
        Callback for GStreamer bus messages (errors, warnings, EOS).
        """
        t = message.type
        if t == Gst.MessageType.EOS:
            self.publish_status("GStreamer: End-Of-Stream reached.")
        elif t == Gst.MessageType.ERROR:
            err, debug = message.parse_error()
            self.publish_status(f"GStreamer ERROR: {err.message} from {message.src.get_name()}. Debug: {debug}")
            global gst_loop
            if gst_loop:
                self.publish_status("FATAL: Critical GStreamer error, shutting down node.")
                gst_loop.quit()
        elif t == Gst.MessageType.WARNING:
            err, debug = message.parse_warning()
            self.publish_status(f"GStreamer WARNING: {err.message} from {message.src.get_name()}. Debug: {debug}")
        return True

    def cleanup_stuck_camera(self):
        """
        Attempts to find and terminate any existing processes that might be holding
        the camera device, preventing GStreamer from starting.
        """
        self.get_logger().info("[CLEANUP] Attempting to clean up any stuck camera processes...")
        try:
            cmd = "pgrep -l -f 'gst-launch-1.0|python.*csi_capture.py|nvarguscamerasrc'"
            output = subprocess.check_output(cmd, shell=True).decode('utf-8')

            pids_to_kill = []
            for line in output.splitlines():
                if line.strip():
                    pid = int(line.split(' ')[0])
                    if pid != os.getpid():
                        pids_to_kill.append(pid)

            if pids_to_kill:
                self.get_logger().warn(f"[CLEANUP] Found potentially stuck camera processes with PIDs: {pids_to_kill}")
                for pid in pids_to_kill:
                    try:
                        os.kill(pid, signal.SIGTERM)
                        self.get_logger().info(f"[CLEANUP] Sent SIGTERM to PID {pid}")
                    except OSError as e:
                        self.get_logger().warn(f"[CLEANUP] Could not send SIGTERM to PID {pid}: {e}")

                time.sleep(1)

                for pid in pids_to_kill:
                    if os.path.exists(f"/proc/{pid}"):
                        self.get_logger().warn(f"[CLEANUP] PID {pid} is still alive after SIGTERM, sending SIGKILL.")
                        try:
                            os.kill(pid, signal.SIGKILL)
                            self.get_logger().info(f"[CLEANUP] Sent SIGKILL to PID {pid}")
                        except OSError as e:
                            self.get_logger().error(f"[CLEANUP] Failed to send SIGKILL to PID {pid}: {e}")
            else:
                self.get_logger().info("[CLEANUP] No obvious stuck camera processes found.")

        except subprocess.CalledProcessError:
            self.get_logger().info("[CLEANUP] No matching processes found by pgrep.")
        except Exception as e:
            self.get_logger().error(f"[CLEANUP] An error occurred during camera cleanup: {e}")

    def gst_thread_main(self):
        """
        Main function for the GStreamer pipeline, running in a separate thread.
        Initializes GStreamer, sets up the pipeline, and runs the GLib main loop.
        """
        global pipeline, bus, gst_loop, csv_file, csv_writer

        self.cleanup_stuck_camera()
        Gst.init(None)

        # Use the parameter-configured FRAME_SAVE_DIR
        if not os.path.exists(self.frame_save_dir):
            try:
                os.makedirs(self.frame_save_dir)
                self.publish_status(f"INFO: Created directory: {self.frame_save_dir}")
            except OSError as e:
                self.publish_status(f"FATAL: Failed to create frame save directory {self.frame_save_dir}: {e}")
                return

        # Use the parameter-configured CSV_FILENAME
        try:
            csv_file = open(self.csv_filename, 'w', newline='')
            csv_writer = csv.writer(csv_file)
            csv_writer.writerow(["FrameNumber", "Timestamp_nanoseconds_relative_camera",
                                 "Timestamp_nanoseconds_absolute_epoch", "Datetime_HumanReadable_Local",
                                 "Image_Filename"])
            self.publish_status(f"INFO: CSV file '{self.csv_filename}' opened for writing.")
        except IOError as e:
            self.publish_status(f"FATAL: Failed to open CSV file {self.csv_filename}: {e}")
            return

        try:
            # Use the parameter-configured FPS
            pipeline_str = (
                f"nvarguscamerasrc ! video/x-raw(memory:NVMM),width=1920,height=1080 ! "
                f"nvvidconv ! video/x-raw ! videorate ! video/x-raw,framerate={int(self.fps)}/1 ! "
                f"nvvidconv ! video/x-raw,format=RGBA ! appsink name=mysink"
            )
            self.publish_status(f"INFO: Attempting to launch GStreamer pipeline: {pipeline_str}")
            pipeline = Gst.parse_launch(pipeline_str)

            appsink = pipeline.get_by_name("mysink")
            if not appsink:
                raise RuntimeError("Could not get appsink element. Check GStreamer pipeline string for 'appsink name=mysink'.")

            appsink.set_property("emit-signals", True)
            appsink.set_property("sync", False)
            appsink.connect("new-sample", self.on_new_sample)

            bus = pipeline.get_bus()
            bus.add_signal_watch()
            bus.connect("message", self.on_message)

            if self.start_waypoint_index == -1:
                self.publish_status("INFO: Start waypoint not set (defaulting to -1), starting capture immediately.")
                ret = pipeline.set_state(Gst.State.PLAYING)
                if ret == Gst.StateChangeReturn.FAILURE:
                    raise RuntimeError("Failed to set pipeline to PLAYING state at startup. GStreamer error.")
                self.is_capturing = True
                self.publish_status("GStreamer Pipeline **STARTED** immediately.")
            else:
                ret = pipeline.set_state(Gst.State.NULL)
                if ret == Gst.StateChangeReturn.FAILURE:
                    raise RuntimeError("Failed to set pipeline to initial NULL state. GStreamer error.")
                self.is_capturing = False
                self.publish_status(f"INFO: GStreamer Pipeline initialized to NULL state. "
                                    f"Waiting for waypoint {self.start_waypoint_index} to start.")

            gst_loop = GLib.MainLoop()
            gst_loop.run()

        except Exception as e:
            self.publish_status(f"FATAL: GStreamer thread encountered an unhandled error during setup or main loop: {e}")
        finally:
            if pipeline:
                self.publish_status("INFO: Cleaning up GStreamer pipeline.")
                pipeline.set_state(Gst.State.NULL)
            if csv_file:
                csv_file.close()
                self.publish_status(f"INFO: CSV file '{self.csv_filename}' closed.")


def main(args=None):
    """
    Main function for the ROS2 node.
    Initializes rclpy, creates the CameraCaptureNode, and spins the node.
    Handles graceful shutdown on Ctrl+C.
    """
    rclpy.init(args=args)
    node = CameraCaptureNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.publish_status("INFO: ROS shutdown signal received (Ctrl+C).")
        pass
    finally:
        global gst_loop
        if gst_loop:
            node.publish_status("INFO: ROS node shutting down. Signaling GStreamer loop to quit.")
            gst_loop.quit()

        if node.gst_thread.is_alive():
            node.gst_thread.join(timeout=2)

        node.publish_status("INFO: Camera Capture ROS Node stopped gracefully.")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
