#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import os
import json
from pathlib import Path
from collections import Counter
import math 
import pandas as pd
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, HistoryPolicy
from mavros_msgs.msg import WaypointReached
import numpy as np
import re
from shapely.geometry import Point, Polygon
from sklearn.cluster import DBSCAN

class Cluster(Node):
    """
    ROS 2 Node for clustering object detection results from a CSV file.
    It listens for waypoint changes, and upon reaching a specified stop waypoint,
    it reads inference data, filters points within a bounding box, and performs
    DBSCAN clustering to identify distinct object locations.
    """
    def __init__(self):
        super().__init__('cluster')

        # Declare parameters with default values
        self.declare_parameter('folder_path', '/tmp/camera_images')
        self.declare_parameter('stop_waypoint_index', -1)
        self.declare_parameter('do_drop', False) # Renamed from original 'do_drop' for clarity

        # Retrieve parameter values
        self.base_directory = self.get_parameter('folder_path').get_parameter_value().string_value
        self.stop_waypoint_index = self.get_parameter('stop_waypoint_index').get_parameter_value().integer_value
        self.enable_clustering_at_waypoint = self.get_parameter('do_drop').value # Use a more descriptive name

        # Construct paths
        self.image_directory = os.path.join(self.base_directory, "captured_frames")
        self.csv_path = os.path.join(self.base_directory, "inference.csv")
        self.crops_dir = os.path.join(self.base_directory, "crops")
        self.clustered_results_path = os.path.join(self.base_directory, "clustered_results.csv")

        # ROS 2 Subscriptions
        self.create_subscription(WaypointReached, '/mavros/mission/reached', self.waypoint_callback, 10)
        
        # QoS profile for reliable communication
        qos_profile_system_default = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )        
        self.create_subscription(
            String,
            '/drone_status/last_waypoint_index',
            self.waypoint_index_callback,
            qos_profile_system_default
        )

        # Define target classes for clustering. Using a list for simplicity.
        self.TARGET_CLASSES = ['person'] 
        
        # Current waypoint index, initialized to -1
        self.current_waypoint_index = -1

        # Expected columns in the inference CSV file
        self.csv_columns = [
            'Image Name', 'Image Path', 'Crop Name', 'Crop Path',
            'Pixel Center X Y', 'Latitude', 'Longitude', 'Class',
            'Confidence', 'Timestamp', 'Yaw'
        ]
        
        self.get_logger().info(f"Cluster node initialized.")
        self.get_logger().info(f"Monitoring waypoints for index {self.stop_waypoint_index} to trigger clustering.")
        if not self.enable_clustering_at_waypoint:
            self.get_logger().info("Clustering at waypoint is disabled (do_drop parameter is False).")

    def waypoint_index_callback(self, msg: String):
        """
        Callback function for the /drone_status/last_waypoint_index topic.
        Updates the stop_waypoint_index dynamically based on received data.
        """
        try:
            last_waypoint_index_int = int(msg.data)
            self.get_logger().info(f'Received last waypoint index (int): {last_waypoint_index_int}. Updating stop_waypoint_index.')
            self.stop_waypoint_index = last_waypoint_index_int
        except ValueError:
            self.get_logger().error(f'Failed to convert received data "{msg.data}" to integer. Is the publisher sending valid integer strings?')
        except Exception as e:
            self.get_logger().error(f'An unexpected error occurred in waypoint_index_callback: {e}')

    def waypoint_callback(self, msg: WaypointReached):
        """
        Callback function for the /mavros/mission/reached topic.
        Checks if the current waypoint matches the stop_waypoint_index and
        triggers the clustering process if 'enable_clustering_at_waypoint' is True.
        """
        new_waypoint_index = msg.wp_seq

        if new_waypoint_index != self.current_waypoint_index:
            old_wp = self.current_waypoint_index
            self.current_waypoint_index = new_waypoint_index

            self.get_logger().info(f"Waypoint changed from {old_wp} to {new_waypoint_index}.")

            if new_waypoint_index == self.stop_waypoint_index and self.enable_clustering_at_waypoint:
                self.get_logger().info(f"Detected STOP WAYPOINT ({new_waypoint_index}). Starting Clustering.")
                self.start_cluster() # Call the method that orchestrates clustering
            else:
                self.get_logger().info(f"Currently at waypoint {new_waypoint_index}. Waiting for stop ({self.stop_waypoint_index}). Clustering is {'enabled' if self.enable_clustering_at_waypoint else 'disabled'}.")

    def start_cluster(self):
        """
        Initiates the clustering process with predefined parameters.
        This method acts as an entry point for the clustering logic.
        """
        # Define clustering parameters
        threshold = 3.0 # Maximum distance for clustering in feet
        threshold_step = 0.5 # Step for iterative clustering
        use_grid = True # Whether to filter points within a bounding box

        self.get_logger().info("Starting clustering process...")
        clustered_csv_path = self.cluster_data(threshold, threshold_step, use_grid)
        if clustered_csv_path:
            self.get_logger().info(f"Clustering complete. Results saved to: {clustered_csv_path}. Starting Drop")
            self.start_drop()
        else:
            self.get_logger().warn("Clustering completed, but no results were saved.")

    def cluster_data(self, threshold: float, threshold_step: float, use_grid: bool = True):
        """
        Performs DBSCAN clustering on the inference data.
        
        Args:
            threshold (float): The maximum distance (in feet) for points to be considered
                                as part of the same cluster.
            threshold_step (float): The increment for the clustering threshold in each iteration.
            use_grid (bool): If True, filters points within a predefined bounding box.
        
        Returns:
            str or None: The path to the saved clustered results CSV, or None if no results.
        """
        # Define the bounding box coordinates (Latitude, Longitude)
        # This defines the area of interest for clustering.
        bounding_box_coords = [
            (13.030947, 77.565240),
            (13.031217, 77.565284),
            (13.031195, 77.565582),
            (13.031080, 77.565680),
            (13.030896, 77.565680),
            (13.030883, 77.565623),
            (13.030843, 77.565613),
            (13.030870, 77.565319),
            (13.030905, 77.565300)
        ]
        bounding_box = Polygon(bounding_box_coords)
        
        # Read the inference CSV file
        try:
            df = pd.read_csv(self.csv_path)
            self.get_logger().info(f"Successfully loaded inference data from {self.csv_path}")
        except FileNotFoundError:
            self.get_logger().error(f"Inference CSV file not found at {self.csv_path}. Cannot perform clustering.")
            return None
        except Exception as e:
            self.get_logger().error(f"Error reading CSV file {self.csv_path}: {e}")
            return None
        
        # Ensure 'Class' column is treated as string and remove leading/trailing spaces
        df['Class'] = df['Class'].astype(str).str.strip()

        # Remove rows where "Class" contains only a confidence score (e.g., "(0.36)")
        # This regex ensures we only keep rows with actual class names.
        df = df[~df['Class'].str.match(r'^\(\d+(\.\d+)?\)$')]
        
        final_results = [] # List to store the final clustered results

        # Process each target class separately
        for target_class in self.TARGET_CLASSES:
            self.get_logger().info(f"\nProcessing class: {target_class}")
            
            # Filter rows that contain the target class name
            class_df = df[df['Class'].str.contains(target_class, regex=False)]
            
            if class_df.empty:
                self.get_logger().info(f"No data found for class: {target_class}")
                continue
            
            # Extract relevant data for clustering
            latitude = class_df['Latitude'].values
            longitude = class_df['Longitude'].values
            class_labels = class_df['Class'].values
            crop_names = class_df['Crop Name'].values
            
            # Combine numeric data with class and crop name for filtering and later use
            data_with_class = np.column_stack((latitude, longitude, class_labels, crop_names))
            
            # Filter points inside the bounding box if 'use_grid' is True
            if use_grid:
                filtered_data = np.array([
                    point for point in data_with_class 
                    if bounding_box.contains(Point(float(point[0]), float(point[1])))
                ])
                
                if filtered_data.size == 0:
                    self.get_logger().info(f"No points for {target_class} are inside the bounding box.")
                    continue
            else:
                filtered_data = data_with_class
            
            # Iterative clustering with increasing threshold
            thr = threshold_step
            all_centroids = []
            
            while thr <= threshold:
                # Convert feet to degrees for epsilon (DBSCAN distance parameter)
                # These conversion factors are approximations.
                feet_to_degrees_lat = thr / 364000 # Approx. degrees per foot latitude
                feet_to_degrees_lon = thr / 288200 # Approx. degrees per foot longitude
                eps = np.mean([feet_to_degrees_lat, feet_to_degrees_lon]) # Average for epsilon

                # Apply DBSCAN clustering
                # min_samples=1 means every point forms a cluster if no other points are within eps.
                dbscan = DBSCAN(eps=eps, min_samples=1)
                dbscan.fit(filtered_data[:, :2].astype(float)) # Cluster on (Latitude, Longitude)
                
                labels = dbscan.labels_
                unique_labels = np.unique(labels)
                
                centroids = []
                class_lists_for_centroids = []
                best_classes = []
                best_crop_names = []
                
                # Process each identified cluster
                for label in unique_labels:
                    cluster_points = filtered_data[labels == label]  
                    mean_centroid = cluster_points[:, :2].astype(float).mean(axis=0) # Calculate mean centroid
                    
                    # Find the actual data point closest to the mean centroid
                    # This ensures the centroid is a real observed location.
                    coords = cluster_points[:, :2].astype(float)
                    distances = np.sqrt(((coords - mean_centroid) ** 2).sum(axis=1))
                    closest_idx = np.argmin(distances)
                    closest_point = coords[closest_idx]
                    
                    # Aggregate class and crop name information for the cluster
                    flat_class_list = []
                    crop_name_list = []
                    for class_item, crop_item in zip(cluster_points[:, 2], cluster_points[:, 3]):
                        # Assuming class_item is always a string like "class_name (confidence)"
                        flat_class_list.append(class_item)
                        crop_name_list.append(crop_item)
                    
                    # Remove duplicate class entries for the "All Classes" column
                    unique_class_list = list(dict.fromkeys(flat_class_list)) # Preserves order
                    
                    # Determine the "Best Class" and its "Crop Name" for the cluster
                    # This identifies the most common class and its associated crop image.
                    class_counts = Counter([re.sub(r'\s*\(.*?\)', '', cls).strip() for cls in flat_class_list])
                    most_common_class, count = class_counts.most_common(1)[0]
                    
                    if count > 1:
                        # If multiple instances of the most common class, average their scores
                        scores = []
                        best_crop = None
                        for cls, crop in zip(flat_class_list, crop_name_list):
                            base_name = re.sub(r'\s*\(.*?\)', '', cls).strip()
                            if base_name == most_common_class:
                                score = re.search(r'\((.*?)\)', cls)
                                if score:
                                    scores.append(float(score.group(1)))
                                if best_crop is None: # Take the first crop name associated with the most common class
                                    best_crop = crop
                        avg_score = sum(scores) / len(scores) if scores else 0
                        best_class = f"{most_common_class} ({avg_score:.2f})"
                    else:
                        # If only one instance or all unique, pick the one with the highest score
                        best_idx = 0
                        best_score = -1.0 # Initialize with a value lower than any possible score
                        for i, cls in enumerate(flat_class_list):
                            score_match = re.search(r'\((.*?)\)', cls)
                            if score_match:
                                score = float(score_match.group(1))
                                if score > best_score:
                                    best_score = score
                                    best_idx = i
                        best_class = flat_class_list[best_idx]
                        best_crop = crop_name_list[best_idx]
                    
                    # Store results for this cluster
                    centroids.append(closest_point)
                    class_lists_for_centroids.append(unique_class_list)
                    best_classes.append(best_class)
                    best_crop_names.append(best_crop)
                
                centroids = np.array(centroids)
                all_centroids.append(centroids)
                final_class_lists = class_lists_for_centroids
                
                # For the next iteration, use the newly calculated centroids as input data
                # This allows for hierarchical clustering if desired.
                filtered_data = np.array(list(zip(
                    centroids[:, 0], centroids[:, 1], 
                    final_class_lists, best_crop_names
                )), dtype=object)
                
                thr += threshold_step # Increment threshold for next iteration
            
            # Add the final centroids for this class to the overall results
            if len(centroids) > 0:
                self.get_logger().info(f"Found {len(centroids)} centroids for {target_class}")
                for idx in range(len(centroids)):
                    final_results.append({
                        "Latitude": centroids[idx][0],
                        "Longitude": centroids[idx][1],
                        "Crop Name": best_crop_names[idx],
                        "Best Class": best_classes[idx],
                        "All Classes": str(final_class_lists[idx]) # Convert list to string for CSV
                    })
            else:
                self.get_logger().info(f"No centroids found for {target_class}")

        # Save combined results to a new CSV file
        if final_results:
            pd.DataFrame(final_results).to_csv(self.clustered_results_path, index=False)
            self.get_logger().info(f"Clustering results saved to {self.clustered_results_path}")
            return self.clustered_results_path
        else:
            self.get_logger().warn("No results to save after clustering.")
            return None

def main(args=None):
    rclpy.init(args=args)
    node = Cluster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
