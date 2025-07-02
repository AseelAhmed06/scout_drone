#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# ROS 2 Imports
import rclpy
from rclpy.node import Node
from rclpy.task import Future # For waiting on service responses
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import Waypoint # Assuming this message is available in ROS 2 mavros_msgs
from mavros_msgs.srv import ParamSet, WaypointPush # Assuming these services are available in ROS 2 mavros_msgs
import subprocess

# Python standard library imports
import os
import xml.etree.ElementTree as ET
import numpy as np
import math
import json
import traceback
import time
# Third-party imports
from shapely.geometry import Polygon, LineString, Point
from shapely.ops import unary_union
# Added for oriented minimum bounding rectangle (though now using find_angle_of_longest_side)
from shapely.geometry import box
from shapely.affinity import rotate
# QoS imports
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data


# --- Core Planning Functions ---
# These functions are kept separate from the ROS class for clarity.

def parse_kml_polygon(kml_file_path):
    """
    Parse KML file and extract polygon coordinates.
    Validates that the KML contains a valid polygon.
    Returns: Shapely Polygon object
    """
    if not os.path.exists(kml_file_path):
        raise IOError(f"KML file not found at path: {kml_file_path}")

    tree = ET.parse(kml_file_path)
    root = tree.getroot()
    
    # Handle KML namespace
    namespace = {'kml': 'http://www.opengis.net/kml/2.2'}
    
    # Find coordinates element
    coordinates_elem = root.find('.//kml:coordinates', namespace)
    if coordinates_elem is None:
        coordinates_elem = root.find('.//coordinates') # Try without namespace
    
    if coordinates_elem is None:
        raise ValueError("No <coordinates> tag found in KML file.") # Consistent error message
    
    # Parse coordinates string
    coord_string = coordinates_elem.text.strip()
    coordinates = []
    
    for coord in coord_string.split():
        if coord:
            try:
                parts = coord.split(',')
                if len(parts) >= 2:
                    lon, lat = float(parts[0]), float(parts[1])
                    coordinates.append((lon, lat))
            except (ValueError, IndexError):
                print(f"WARN: Skipping malformed coordinate: {coord}")
                continue
    
    if len(coordinates) < 3:
        raise ValueError(f"A valid polygon requires at least 3 vertices. Found {len(coordinates)}.")

    return Polygon(coordinates)

def find_closest_point_on_polygon(home_coords, polygon):
    """
    Find the closest point on the polygon boundary to home coordinates
    """
    home_point = Point(home_coords[1], home_coords[0])  # lon, lat
    
    # If home is inside polygon, return home coordinates
    if polygon.contains(home_point):
        return home_coords
    
    # Find closest point on polygon boundary
    closest_point = polygon.exterior.interpolate(polygon.exterior.project(home_point))
    return [closest_point.y, closest_point.x]  # lat, lon

def find_angle_of_longest_side(polygon):
    """
    Calculates the angle (in degrees, 0-180) of the longest edge of the polygon.
    This angle is then used to orient the lawnmower pattern.
    """
    max_length = 0
    longest_side_angle = 0
    
    coords = list(polygon.exterior.coords)
    # Iterate through segments (edges) of the polygon
    for i in range(len(coords) - 1):
        p1 = coords[i]
        p2 = coords[i+1]
        
        # Calculate length of the segment
        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]
        length = math.sqrt(dx**2 + dy**2)
        
        if length > max_length:
            max_length = length
            # Calculate angle in radians using atan2 (handles all quadrants)
            angle_rad = math.atan2(dy, dx)
            # Convert to degrees
            angle_deg = math.degrees(angle_rad)
            
            # Normalize the angle to be within the 0-180 degree range
            # This ensures that angles like -45 and 135 (which represent the same slope)
            # are treated consistently for parallel lines.
            longest_side_angle = angle_deg % 180
            if longest_side_angle < 0:
                longest_side_angle += 180
                
    return longest_side_angle

def generate_lawnmower_pattern_internal(polygon, line_spacing=0.0001, angle=0, buffer_distance=0.00007):
    """
    Internal function to generate lawnmower pattern, used for optimization.
    """
    # Create buffer zone inside polygon (negative buffer shrinks polygon)
    buffered_polygon = polygon.buffer(-buffer_distance)
    
    # If buffer is too large, polygon might disappear, in which case use original polygon
    if buffered_polygon.is_empty:
        print("WARN: Polygon is too small for the given buffer distance. Using original polygon.") # Fallback print
        buffered_polygon = polygon
    
    # Get buffered polygon bounds
    minx, miny, maxx, maxy = buffered_polygon.bounds
    
    # Convert angle to radians
    angle_rad = math.radians(angle)
    
    # Calculate the diagonal of the bounding box, which provides a safe length
    # to extend lines beyond the polygon's boundaries to ensure full coverage.
    diagonal = math.sqrt((maxx - minx)**2 + (maxy - miny)**2)
    
    # Add a generous margin to ensure complete coverage, especially for irregular shapes.
    margin = line_spacing * 3 
    
    # List to hold the generated lines
    lines = []
    
    # Calculate number of lines needed with extra coverage
    # Handle near-horizontal lines (angle close to 0 or 180 degrees)
    if abs(angle_rad) < 0.001 or abs(angle_rad - math.pi) < 0.001:  # Horizontal lines
        extended_miny = miny - margin
        extended_maxy = maxy + margin
        coverage_distance = extended_maxy - extended_miny
        # Add extra lines for safety to prevent gaps
        num_lines = int(math.ceil(coverage_distance / line_spacing)) + 3 
        
        for i in range(num_lines):
            y = extended_miny + i * line_spacing
            # Create a very long horizontal line to ensure it crosses the entire buffered polygon
            line = LineString([(minx - diagonal, y), (maxx + diagonal, y)])
            lines.append(line)
            
    # Handle near-vertical lines (angle close to 90 or 270 degrees)
    elif abs(angle_rad - math.pi/2) < 0.001 or abs(angle_rad - 3*math.pi/2) < 0.001:  # Vertical lines
        extended_minx = minx - margin
        extended_maxx = maxx + margin
        coverage_distance = extended_maxx - extended_minx
        # Add extra lines for safety
        num_lines = int(math.ceil(coverage_distance / line_spacing)) + 3  
        
        for i in range(num_lines):
            x = extended_minx + i * line_spacing
            # Create a very long vertical line
            line = LineString([(x, miny - diagonal), (x, maxy + diagonal)])
            lines.append(line)
            
    else:  # Angled lines (general case)
        # Calculate the angle perpendicular to the survey lines to define line positions
        perp_angle = angle_rad + math.pi/2
        
        # Project polygon points onto the perpendicular axis to find the range for lines
        cos_perp = math.cos(perp_angle)
        sin_perp = math.sin(perp_angle)
        
        # Get all polygon points and project them
        coords = list(buffered_polygon.exterior.coords)
        projections = [x * cos_perp + y * sin_perp for x, y in coords]
        min_proj = min(projections) - margin
        max_proj = max(projections) + margin
        
        coverage_distance = max_proj - min_proj
        # Add extra lines for safety
        num_lines = int(math.ceil(coverage_distance / line_spacing)) + 3  
        
        for i in range(num_lines):
            proj_pos = min_proj + i * line_spacing
            
            # Line direction vector based on the survey angle
            dx = math.cos(angle_rad)
            dy = math.sin(angle_rad)
            
            # Calculate the center of the bounding box
            center_x = (minx + maxx) / 2
            center_y = (miny + maxy) / 2
            
            # Find a point on the perpendicular line corresponding to proj_pos
            px = center_x + (proj_pos - (center_x * cos_perp + center_y * sin_perp)) * cos_perp
            py = center_y + (proj_pos - (center_x * cos_perp + center_y * sin_perp)) * sin_perp
            
            # Create a long line passing through (px, py) with the desired angle
            line = LineString([
                (px - diagonal * dx, py - diagonal * dy),
                (px + diagonal * dx, py + diagonal * dy)
            ])
            lines.append(line)
    
    # Intersect each generated long line with the buffered polygon to get survey segments
    segments = []
    for line in lines:
        intersection = buffered_polygon.intersection(line)
        if intersection.is_empty:
            continue
        # If intersection results in multiple line segments (e.g., for complex polygons)
        elif hasattr(intersection, 'geoms'):  
            for geom in intersection.geoms:
                if isinstance(geom, LineString) and len(geom.coords) >= 2:
                    segments.append(geom)
        # If intersection results in a single line segment
        elif isinstance(intersection, LineString) and len(intersection.coords) >= 2:
            segments.append(intersection)
    
    # Remove any segments that are too short (often artifacts of intersection)
    min_segment_length = line_spacing * 0.05  # More sensitive filtering
    segments = [seg for seg in segments if seg.length > min_segment_length]
    
    # Sort segments to create a coherent path
    if abs(angle_rad) < 0.001 or abs(angle_rad - math.pi) < 0.001:  # Horizontal
        segments.sort(key=lambda seg: seg.coords[0][1])  # Sort by y-coordinate
    elif abs(angle_rad - math.pi/2) < 0.001 or abs(angle_rad - 3*math.pi/2) < 0.001:  # Vertical
        segments.sort(key=lambda seg: seg.coords[0][0])  # Sort by x-coordinate
    else:  # Angled - sort by projection onto perpendicular axis
        cos_perp = math.cos(angle_rad + math.pi/2)
        sin_perp = math.sin(angle_rad + math.pi/2)
        segments.sort(key=lambda seg: seg.coords[0][0] * cos_perp + seg.coords[0][1] * sin_perp)
    
    return segments

def generate_lawnmower_pattern(polygon, line_spacing=0.0001, angle=None, buffer_distance=0.00007, home_coords=None, optimize_angle=True):
    """
    Generate lawnmower pattern within polygon with buffer from boundary.
    The pattern will always be parallel to the longest side of the polygon if `angle` is None.
    
    Args:
        polygon: Shapely Polygon object
        line_spacing: Distance between parallel lines (in degrees for lat/lon)
        angle: (Optional) Angle of the pattern in degrees (0 = east-west lines).
                If None, the angle is automatically calculated to be parallel to the polygon's longest side.
        buffer_distance: Distance to stay away from polygon boundary (in degrees)
        home_coords: [lat, lon] of home position to optimize starting point
        optimize_angle: Boolean, if True and angle is None, uses find_angle_of_longest_side.
                        If False, and angle is None, defaults to 0.
    
    Returns:
        List of LineString objects representing the flight path
    """
    # Use actual_angle for the planning logic, which might be replaced by optimization
    actual_angle = angle 

    if optimize_angle:
        if actual_angle == LawnmowerPlannerNode.DEFAULT_OPTIMIZE_ANGLE_SENTINEL: # Check for sentinel
            print("INFO: optimize_angle is True, and sentinel angle was provided. Finding optimal survey angle (aligned with longest side)...")
            actual_angle = find_angle_of_longest_side(polygon)
            print(f"INFO: Optimal angle found: {actual_angle:.2f}°")
        else:
            print(f"INFO: optimize_angle is True, but a specific angle ({actual_angle}) was provided. Using provided angle.")
    else: # optimize_angle is False
        if actual_angle == LawnmowerPlannerNode.DEFAULT_OPTIMIZE_ANGLE_SENTINEL: # Check for sentinel
            actual_angle = 0.0
            print("INFO: optimize_angle is False, and sentinel angle was provided. Defaulting to 0 degrees.")
        else:
            print(f"INFO: Using provided angle: {actual_angle} degrees (optimize_angle is False).")

    # Generate segments using the internal function with the determined actual_angle
    segments = generate_lawnmower_pattern_internal(polygon, line_spacing, actual_angle, buffer_distance)
    
    if not segments:
        print("Warning: No valid survey lines generated")
        return []
    
    # If home coordinates provided, optimize starting point
    if home_coords and segments:
        # The optimize_starting_point function doesn't seem to use angle_rad internally
        # but if it did, it would be passed here. For now, pass actual_angle if needed.
        segments, reverse_first = optimize_starting_point(segments, home_coords, math.radians(actual_angle))
    else:
        reverse_first = False
    
    # Create alternating lawnmower pattern with optimized connections
    flight_lines = []
    for i, segment in enumerate(segments):
        coords = list(segment.coords)
        
        # Handle alternating pattern with potential first line reversal
        should_reverse = (i % 2 == 1)
        if reverse_first:
            should_reverse = not should_reverse
            
        if should_reverse:
            coords.reverse()
            
        flight_lines.append(LineString(coords))
    
    return flight_lines

def optimize_starting_point(segments, home_coords, angle_rad): # angle_rad now explicitly passed
    """
    Reorder segments to start closest to home coordinates.
    The `angle_rad` parameter is currently not used in the provided logic,
    but is kept in signature as it was passed by generate_lawnmower_pattern.
    """
    if not segments or not home_coords:
        return segments, False
    
    home_point = Point(home_coords[1], home_coords[0])  # lon, lat
    
    # Find which end of which segment is closest to home
    min_distance = float('inf')
    best_start_idx = 0
    reverse_first = False
    
    for i, segment in enumerate(segments):
        coords = list(segment.coords)
        
        # Check distance to start of segment
        start_point = Point(coords[0])
        dist_to_start = home_point.distance(start_point)
        
        # Check distance to end of segment
        end_point = Point(coords[-1])
        dist_to_end = home_point.distance(end_point)
        
        if dist_to_start < min_distance:
            min_distance = dist_to_start
            best_start_idx = i
            reverse_first = False
        
        if dist_to_end < min_distance:
            min_distance = dist_to_end
            best_start_idx = i
            reverse_first = True
            
    # Reorder segments starting from the closest one
    reordered_segments = segments[best_start_idx:] + segments[:best_start_idx]
    
    return reordered_segments, reverse_first

def generate_geofence_polygon(polygon, buffer_distance_meters=1.0, method='simple_offset', home_coords=None):
    """
    Generate a geofence polygon around the survey area.
    If home_coords are provided, the fence is expanded to include them.
    
    Args:
        polygon: Original survey polygon (Shapely Polygon).
        buffer_distance_meters: Buffer distance in meters.
        method: 'simple_offset', 'curved_buffer', or 'simplified_buffer'.
        home_coords: Optional [lat, lon] to include in the geofence.
        
    Returns:
        Shapely Polygon representing the geofence boundary.
    """
    # The base shape for the geofence starts as the survey polygon
    base_shape = polygon

    # If home coordinates are provided, expand the base shape to include them
    if home_coords:
        home_point = Point(home_coords[1], home_coords[0])  # lon, lat
        # Create a new shape that is the convex hull of the original polygon and the home point
        base_shape = unary_union([polygon, home_point]).convex_hull

    # Convert meters to degrees (rough approximation: 1 degree ≈ 111,000 meters)
    buffer_distance_deg = buffer_distance_meters / 111000.0
    
    if method == 'simple_offset':
        # Creates an offset by extending vertices from the centroid.
        # This maintains the number of vertices of the base_shape.
        fence_coords = []
        coords = list(base_shape.exterior.coords)[:-1]  # Remove duplicate last point
        centroid = base_shape.centroid
        
        for coord in coords:
            dx, dy = coord[0] - centroid.x, coord[1] - centroid.y
            length = math.sqrt(dx*dx + dy*dy)
            if length > 0:
                # Extend vertex outward along the vector from the centroid
                new_x = coord[0] + (dx / length) * buffer_distance_deg
                new_y = coord[1] + (dy / length) * buffer_distance_deg
                fence_coords.append((new_x, new_y))
            else:
                fence_coords.append(coord)
        
        if fence_coords:
            fence_coords.append(fence_coords[0]) # Close the polygon
        fence_polygon = Polygon(fence_coords)
        
    elif method == 'curved_buffer':
        # Standard shapely buffer, creates smooth corners.
        fence_polygon = base_shape.buffer(buffer_distance_deg)
        
    elif method == 'simplified_buffer':
        # A standard buffer followed by simplification to reduce vertex count.
        temp_polygon = base_shape.buffer(buffer_distance_deg)
        fence_polygon = temp_polygon.simplify(buffer_distance_deg * 0.1, preserve_topology=True)
        
    else:
        raise ValueError("Method must be 'simple_offset', 'curved_buffer', or 'simplified_buffer'")
    
    return fence_polygon
def create_waypoints_from_pattern(flight_lines, home_coords=None):
    """
    Convert a list of flight LineString objects into a list of waypoint dictionaries.
    Includes logic for connecting lines and adding home/return points.
    
    Args:
        flight_lines: List of Shapely LineString objects representing the survey path.
        home_coords: Optional [lat, lon] for the home/return point.
        
    Returns:
        List of dictionaries, each representing a waypoint with 'lat', 'lon', and 'type'.
    """
    waypoints = []
    
    # The first waypoint is the start of the first survey line
    if flight_lines:
        first_line_coords = flight_lines[0].coords[0]
        waypoints.append({'lat': first_line_coords[1], 'lon': first_line_coords[0], 'type': 'entry'})

    for line in flight_lines:
        # Add the end point of the current survey line
        end_coord = line.coords[-1]
        waypoints.append({'lat': end_coord[1], 'lon': end_coord[0], 'type': 'survey_end'})

        # If there's a next line, add its starting point to create a connection/turn waypoint
        next_line_index = flight_lines.index(line) + 1
        if next_line_index < len(flight_lines):
            next_line_start = flight_lines[next_line_index].coords[0]
            waypoints.append({'lat': next_line_start[1], 'lon': next_line_start[0], 'type': 'turn_start'})
    
    # Add an exit waypoint back to home if home coordinates are provided
    if home_coords:
        waypoints.append({'lat': home_coords[0], 'lon': home_coords[1], 'type': 'exit'})
    
    return waypoints

def save_waypoints_mission_planner(waypoints, output_file, altitude=50, velocity=10):
    """
    Save waypoints in Mission Planner format (.waypoints).
    Includes Home, Takeoff, Set Speed, Waypoints, and RTL commands.
    
    Args:
        waypoints: List of waypoint dictionaries.
        output_file: Output filename for the mission waypoints.
        altitude: Default altitude for survey waypoints in meters.
        velocity: Cruise velocity for the drone in m/s.
    """
    with open(output_file, 'w') as f:
        f.write("QGC WPL 110\n") # Mission Planner header
        
        # Item 0: Home position (MAV_CMD_NAV_WAYPOINT with IS_CURRENT = 1)
        # This acts as the initial home position for the mission.
        first_wp = waypoints[0] if waypoints else {'lat': 0, 'lon': 0}
        f.write(f"0\t1\t0\t16\t0\t0\t0\t0\t{first_wp['lat']:.7f}\t{first_wp['lon']:.7f}\t{altitude:.6f}\t1\n")
        
        seq = 1 # Start sequence number for mission commands
        
        # Item 1: Takeoff command (MAV_CMD_NAV_TAKEOFF)
        # Sets the target altitude for takeoff.
        f.write(f"{seq}\t0\t3\t22\t0.00000000\t0.00000000\t0.00000000\t0.00000000\t0.00000000\t0.00000000\t{altitude:.6f}\t1\n")
        seq += 1
        
        # Item 2: Set speed command (MAV_CMD_DO_CHANGE_SPEED = 178)
        # Sets the drone's cruise speed.
        f.write(f"{seq}\t0\t3\t178\t1\t{velocity:.1f}\t-1\t0\t0.00000000\t0.00000000\t0.000000\t1\n")
        seq += 1
        
        # Following items: Mission waypoints (MAV_CMD_NAV_WAYPOINT)
        for wp in waypoints:
            f.write(f"{seq}\t0\t3\t16\t0.00000000\t0.00000000\t0.00000000\t0.00000000\t{wp['lat']:.8f}\t{wp['lon']:.8f}\t{altitude:.6f}\t1\n")
            seq += 1
            
        # Final item: Return to launch command (MAV_CMD_NAV_RETURN_TO_LAUNCH)
        # Directs the drone to return to the home position set at item 0.
        f.write(f"{seq}\t0\t3\t20\t0.00000000\t0.00000000\t0.00000000\t0.00000000\t0.00000000\t0.00000000\t0.000000\t1\n")

    print(f"INFO: Saved Mission Planner file to {output_file}") # Fallback print

def save_waypoints_mavlink(waypoints, output_file, altitude=50, velocity=10):
    """
    Save waypoints in MAVLink JSON format (.mission), compatible with QGroundControl.
    
    Args:
        waypoints: List of waypoint dictionaries.
        output_file: Output filename for the mission waypoints.
        altitude: Default altitude for survey waypoints in meters.
        velocity: Cruise velocity for the drone in m/s.
    """
    mission_items = []

    # Home position (MAV_CMD_NAV_WAYPOINT, IsCurrent=1 means it's the home)
    home_wp = waypoints[0] if waypoints else {'lat': 0, 'lon': 0}
    
    # Takeoff command (MAV_CMD_NAV_TAKEOFF = 22)
    mission_items.append({
        "autoContinue": True,
        "command": 22,
        "doJumpId": 1,
        "frame": 3, # MAV_FRAME_GLOBAL_RELATIVE_ALT
        "params": [0, 0, 0, None, home_wp["lat"], home_wp["lon"], altitude],
        "type": "SimpleItem"
    })
    
    # Mission waypoints (MAV_CMD_NAV_WAYPOINT = 16)
    for i, wp in enumerate(waypoints):
        mission_items.append({
            "autoContinue": True,
            "command": 16,
            "doJumpId": i + 2, # Sequence ID for QGC
            "frame": 3, # MAV_FRAME_GLOBAL_RELATIVE_ALT
            "params": [0, 0, 0, None, wp["lat"], wp["lon"], altitude],
            "type": "SimpleItem"
        })
    
    # Final item: Return to launch command (MAV_CMD_NAV_RETURN_TO_LAUNCH = 20)
    mission_items.append({
        "autoContinue": True,
        "command": 20,
        "doJumpId": len(waypoints) + 2, # Adjust jump ID
        "frame": 3, # MAV_FRAME_GLOBAL_RELATIVE_ALT
        "params": [0, 0, 0, None, 0, 0, 0], # Params often null/zero for RTL
        "type": "SimpleItem"
    })

    mavlink_json = {
        "fileType": "Mission",
        "version": 2,
        "groundStation": "QGroundControl",
        "mission": {
            "cruiseSpeed": velocity,
            "hoverSpeed": 5,
            "items": mission_items,
            "plannedHomePosition": [home_wp["lat"], home_wp["lon"], altitude],
            "vehicleType": 2 # MAV_TYPE_QUADROTOR
        },
        "rallyPoints": {
            "points": [],
            "version": 2
        }
    }
    
    with open(output_file, 'w') as f:
        json.dump(mavlink_json, f, indent=4)
    print(f"INFO: Saved MAVLink JSON mission file to {output_file}")

def save_geofence_waypoints(polygon, output_file, buffer_distance_meters=1.0, return_point_coords=None):
    """
    Save geofence waypoints in a Mission Planner-compatible format (.waypoints).
    The geofence will include the return_point_coords if provided.
    
    Args:
        polygon: Original survey polygon.
        output_file: Output filename for the fence waypoints.
        buffer_distance_meters: Buffer distance in meters around the original polygon.
        return_point_coords: Optional [lat, lon] for the return point.
    """
    # Generate the fence polygon, including the return/home point if available
    # Using 'curved_buffer' as a sensible default for geofence visual smoothness
    fence_polygon = generate_geofence_polygon(
        polygon, 
        buffer_distance_meters, 
        home_coords=return_point_coords
    )
    
    # Get exterior coordinates of the fence polygon
    fence_coords = list(fence_polygon.exterior.coords)
    
    # Remove the last coordinate if it's the same as the first to avoid duplicates in output
    if fence_coords and fence_coords[0] == fence_coords[-1]:
        fence_coords = fence_coords[:-1]

    if not fence_coords:
        print("WARN: Could not generate geofence coordinates.") # Fallback print
        return [] # Return empty list if no coords generated
    
    with open(output_file, 'w') as f:
        f.write("QGC WPL 110\n") # Mission Planner header
        
        # Determine the home/return point for the mission
        if return_point_coords:
            home_lat, home_lon = return_point_coords[0], return_point_coords[1]
        else:
            # Default to the first vertex of the fence if no specific home is provided
            home_lon, home_lat = fence_coords[0][0], fence_coords[0][1]

        # Item 0: Home position (MAV_CMD_NAV_WAYPOINT with IS_CURRENT = 1, required for Mission Planner)
        f.write(f"0\t1\t0\t16\t0\t0\t0\t0\t{home_lat:.7f}\t{home_lon:.7f}\t0.000000\t1\n")
        
        # Following items: Fence vertices (MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION)
        vertex_count = len(fence_coords)
        for i, coord in enumerate(fence_coords):
            seq = i + 1 # Sequence number starts after home
            # MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION (5001) is used for geofence points
            # param1 is total vertex count, lat/lon are in param8/9
            f.write(f"{seq}\t0\t3\t5001\t{vertex_count:.8f}\t0.00000000\t0.00000000\t0.00000000\t{coord[1]:.8f}\t{coord[0]:.8f}\t0.000000\t1\n")
            
    print(f"INFO: Saved Geofence file with {len(fence_coords)} vertices to {output_file}") # Fallback print
    return fence_coords # Return the coordinates for possible internal use/logging

def save_pattern_to_kml(flight_lines, output_file, name="Lawnmower Pattern", home_coords=None, fence_polygon=None):
    """
    Save the flight pattern, home point, and geofence to a KML file for visualization in Google Earth.
    
    Args:
        flight_lines: List of LineString objects representing the drone's flight path.
        output_file: Output filename for the KML.
        name: Name to appear in the KML file.
        home_coords: Optional [lat, lon] of the home position to display.
        fence_polygon: Optional Shapely Polygon representing the geofence to display.
    """
    kml_content = f'''<?xml version="1.0" encoding="UTF-8"?>
<kml xmlns="http://www.opengis.net/kml/2.2">
  <Document>
    <name>{name}</name>
    <Style id="flightPath">
      <LineStyle>
        <color>ff0000ff</color> <!-- Red color, opaque -->
        <width>2</width>
      </LineStyle>
    </Style>
    <Style id="homePoint">
      <IconStyle>
        <color>ff00ff00</color> <!-- Green color, opaque -->
        <scale>1.2</scale>
        <Icon>
          <href>http://maps.google.com/mapfiles/kml/pushpin/grn-pushpin.png</href>
        </Icon>
      </IconStyle>
    </Style>
    <Style id="fenceBoundary">
      <LineStyle>
        <color>ffff0000</color> <!-- Blue color, opaque -->
        <width>3</width>
      </LineStyle>
      <PolyStyle>
        <color>30ff0000</color> <!-- Blue color, semi-transparent (alpha 30) -->
        <fill>1</fill>
        <outline>1</outline>
      </PolyStyle>
    </Style>'''
    
    # Add fence polygon if provided
    if fence_polygon:
        fence_coords = list(fence_polygon.exterior.coords)
        # KML coordinates are lon,lat,altitude. Altitude is 0 for ground.
        fence_coords_str = ' '.join([f"{coord[0]},{coord[1]},0" for coord in fence_coords])
        kml_content += f'''
    <Placemark>
      <name>Geofence Boundary</name>
      <styleUrl>#fenceBoundary</styleUrl>
      <Polygon>
        <outerBoundaryIs>
          <LinearRing>
            <coordinates>{fence_coords_str}</coordinates>
          </LinearRing>
        </outerBoundaryIs>
      </Polygon>
    </Placemark>'''
    
    # Add home point if provided
    if home_coords:
        kml_content += f'''
    <Placemark>
      <name>Home Position</name>
      <styleUrl>#homePoint</styleUrl>
      <Point>
        <coordinates>{home_coords[1]},{home_coords[0]},0</coordinates>
      </Point>
    </Placemark>'''
    
    # Create one continuous LineString for the entire flight path for better visualization
    full_path_coords = []
    if flight_lines:
        # Start at home if available, otherwise start at the beginning of the first line
        if home_coords:
            full_path_coords.append( (home_coords[1], home_coords[0]) )
        
        # Add the starting point of the first survey line
        full_path_coords.append( (flight_lines[0].coords[0][0], flight_lines[0].coords[0][1]) )
        
        # Connect all survey lines, including implicit turns between them
        for i in range(len(flight_lines)):
            full_path_coords.extend([(c[0], c[1]) for c in flight_lines[i].coords]) # Add current line's points
            if i < len(flight_lines) - 1:
                # Add the start of the next line to show the connection (turn)
                full_path_coords.append( (flight_lines[i+1].coords[0][0], flight_lines[i+1].coords[0][1]) )

        # End at home if available, completing the path visualization
        if home_coords:
            full_path_coords.append( (home_coords[1], home_coords[0]) )

    if full_path_coords:
        # Format coordinates for KML (lon,lat,altitude)
        coords_str = ' '.join([f"{coord[0]},{coord[1]},0" for coord in full_path_coords])
        kml_content += f'''
    <Placemark>
      <name>Full Flight Path</name>
      <styleUrl>#flightPath</styleUrl>
      <LineString>
        <tessellate>1</tessellate> <!-- Render the path on the terrain -->
        <coordinates>{coords_str}</coordinates>
      </LineString>
    </Placemark>'''

    kml_content += '''
  </Document>
</kml>'''
    
    with open(output_file, 'w') as f:
        f.write(kml_content)
    print(f"INFO: Saved KML visualization to {output_file}") # Fallback print

class LawnmowerPlannerNode(Node):
    DEFAULT_OPTIMIZE_ANGLE_SENTINEL = -999.0 

    def __init__(self):
        super().__init__('lawnmower_planner_node')
        self.get_logger().info("Lawnmower Planner Node Initialized.")

        # Declare all parameters with default values.
        # This makes them accessible via launch files AND get_parameter()
        self.declare_parameter('kml_file', '') 
        self.declare_parameter('output_dir', '/tmp') 
        self.declare_parameter('line_spacing', 0.00018)
        self.declare_parameter('flight_altitude', 50.0)
        self.declare_parameter('flight_velocity', 14.0)
        self.declare_parameter('fence_buffer', 4.0)
        self.declare_parameter('optimize_angle', True)
        self.declare_parameter('fence_script', '/home/aseel/scout_drone/src/drone_scout/drone_scout/pymavlink_fence.py') # Default for fence script
        self.declare_parameter('angle', self.DEFAULT_OPTIMIZE_ANGLE_SENTINEL) 

        # Get parameter values during initialization.
        # These will be the launch file values or the declared defaults.
        self.kml_file_init = self.get_parameter('kml_file').get_parameter_value().string_value
        self.output_dir_init = self.get_parameter('output_dir').get_parameter_value().string_value
        self.fence_script_init = self.get_parameter('fence_script').get_parameter_value().string_value
        
        # Initialize default_params from declared parameters for easy dynamic updates.
        # These are the *initial* values, which can be overridden by topic messages.
        self.default_params = {
            'line_spacing': self.get_parameter('line_spacing').value,
            'flight_altitude': self.get_parameter('flight_altitude').value,
            'flight_velocity': self.get_parameter('flight_velocity').value,
            'fence_buffer': self.get_parameter('fence_buffer').value,
            'optimize_angle': self.get_parameter('optimize_angle').value,
            'angle': self.get_parameter('angle').value 
        }

        self.get_logger().info(f"Initialized output_dir: {self.output_dir_init}")
        self.get_logger().info(f"Initialized kml_file: {self.kml_file_init}")
        self.get_logger().info(f"Initialized fence_script: {self.fence_script_init}")

        # ROS 2 Publishers and Subscribers
        self.trigger_sub = self.create_subscription(String, '/planning/generate_pattern', self.trigger_callback, 1)
        self.status_pub = self.create_publisher(String, '/planning/status', 1)
        self.summary_pub = self.create_publisher(String, '/planning/mission_summary', 1)
        
        # ROS 2 Service Clients
        self.param_set_cli = self.create_client(ParamSet, '/mavros/param/set')
        self.param_get_cli = self.create_client(ParamSet, '/mavros/param/get') 
        self.mission_push_cli = self.create_client(WaypointPush, '/mavros/mission/push')
        self.latest_gps_fix = None

        # Set up a persistent GPS subscriber
        self.gps_sub = self.create_subscription(
            NavSatFix,
            '/mavros/global_position/global',
            self.gps_callback,
            qos_profile_sensor_data  # or use qos_profile_sensor_data
        )

        # Add a small delay before waiting for services
        time.sleep(1.0) # Give DDS a moment for discovery

        # Wait for service clients to be available
        
        self.get_logger().info("Waiting for /mavros/mission/push service...")
        self.mission_push_cli.wait_for_service()
        
        self.get_logger().info("Ready. Waiting for planning request on /planning/generate_pattern topic...")
        
    def gps_callback(self, msg):
        self.latest_gps_fix = msg
        self.get_logger().debug(f"[GPS] lat={msg.latitude}, lon={msg.longitude}, alt={msg.altitude}")

    def set_fence_parameters(self):
        """
        Set fence parameters to enable geofencing
        """
        fence_params = {
            'FENCE_ENABLE': 1,      # Enable fence
            'FENCE_ACTION': 1,      # Action when fence is breached (1=RTL, 2=Land, 3=Brake)
            'FENCE_TYPE': 4,        # Fence type (1=max alt, 2=circle, 4=polygon, 7=all)
            'FENCE_MARGIN': 1.0,    # Margin in meters
            'FENCE_ALT_MAX': 250.0, # Maximum altitude (kept from previous version)
        }
        
        for param_name, param_value in fence_params.items():
            req = ParamSet.Request()
            req.param_id = param_name
            if isinstance(param_value, int):
                req.integer = param_value
                req.real = 0.0 
            elif isinstance(param_value, float):
                req.real = param_value
                req.integer = 0 
            else:
                self.get_logger().warn(f"Unsupported parameter type for {param_name}: {type(param_value)}. Skipping.")
                continue

            self.get_logger().info(f"Setting parameter {param_name} to {param_value}")
            future = self.param_set_cli.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            
            if future.result() is not None:
                if future.result().success:
                    self.get_logger().info(f"Successfully set {param_name}")
                else:
                    self.get_logger().warn(f"Failed to set {param_name}")
            else:
                self.get_logger().error(f"Service call for {param_name} failed: No response.")
            time.sleep(0.1)  # Small delay between parameter sets
    
    def parse_waypoint_file(self, file_path):
        """
        Parses a QGC WPL file into a list of mavros_msgs.msg.Waypoint objects.
        """
        waypoints = []
        try:
            with open(file_path, 'r') as file:
                lines = file.readlines()
        except FileNotFoundError:
            self.get_logger().error(f"Waypoint file not found at {file_path}")
            return []

        if not lines or not lines[0].startswith("QGC WPL"):
            self.get_logger().error("Invalid or empty waypoint file.")
            return []

        for line_num, line in enumerate(lines[1:]):
            parts = line.strip().split('\t')
            if len(parts) < 12:
                self.get_logger().warn(f"Skipping malformed waypoint line {line_num + 1}: {line.strip()}")
                continue
            try:
                wp = Waypoint()
                wp.is_current = bool(int(parts[1]))
                wp.frame = int(parts[2])
                wp.command = int(parts[3])
                wp.param1 = float(parts[4])
                wp.param2 = float(parts[5])
                wp.param3 = float(parts[6])
                wp.param4 = float(parts[7])
                wp.x_lat = float(parts[8])
                wp.y_long = float(parts[9])
                wp.z_alt = float(parts[10])
                wp.autocontinue = bool(int(parts[11]))
                waypoints.append(wp)
            except ValueError as e:
                self.get_logger().error(f"Error parsing waypoint line {line_num + 1}: {line.strip()} - {e}")
                continue

        self.get_logger().info(f"Loaded {len(waypoints)} waypoints from file.")
        return waypoints

    def push_mission(self, waypoints):
        self.get_logger().info(f"Attempting to push {len(waypoints)} waypoints to FCU...")
        req = WaypointPush.Request()
        req.start_index = 0
        req.waypoints = waypoints

        future = self.mission_push_cli.call_async(req)

        # Wait with timeout
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

        if future.done():
            resp = future.result()
            if resp:
                self.get_logger().info(f"Push response: success={resp.success}, transferred={resp.wp_transfered}, error_code={resp.error_code}")
                if resp.success or resp.wp_transfered > 0:
                    self.get_logger().info("Mission push completed successfully.")
                else:
                    self.get_logger().error("Mission push failed.")
            else:
                self.get_logger().error("Service call completed but response is None.")
        else:
            self.get_logger().error("Mission push service call timed out. FCU may have received waypoints but didn't acknowledge in time.")


    def get_home_coordinates(self, timeout_sec=30.0):
        self.get_logger().info("Waiting for GPS fix (home coordinates)...")
        
        check_interval = 0.5  # seconds
        waited = 0.0

        while rclpy.ok() and waited < timeout_sec:
            if self.latest_gps_fix and self.latest_gps_fix.status.status >= 0:
                lat = self.latest_gps_fix.latitude
                lon = self.latest_gps_fix.longitude
                self.get_logger().info(f"Home coordinates acquired: lat={lat}, lon={lon}")
                return [lat, lon]
            
            time.sleep(check_interval)
            waited += check_interval

        self.get_logger().warn("Timeout: Could not get valid GPS fix. Using fallback coordinates.")
        return None  # Example fallback (Bangalore)


    def trigger_callback(self, msg):
        self.get_logger().info("Received planning request.")
        status_msg = String()
        status_msg.data = "Received planning request."
        self.status_pub.publish(status_msg)
        
        # Start with default parameters from node initialization
        current_planning_params = self.default_params.copy()
        
        # KML file path and output directory to be used for this planning run
        kml_file_path_to_use = self.kml_file_init 
        output_dir_to_use = self.output_dir_init
        fence_script_to_use = self.fence_script_init

        try:
            request_data = msg.data
            parsed_params = json.loads(request_data)
            
            # Override parameters from the JSON message if provided
            if 'kml_file' in parsed_params:
                kml_file_path_to_use = parsed_params.pop('kml_file')
                self.get_logger().info(f"KML file path overridden by topic: {kml_file_path_to_use}")
            
            if 'output_dir' in parsed_params:
                output_dir_to_use = parsed_params.pop('output_dir')
                self.get_logger().info(f"Output directory overridden by topic: {output_dir_to_use}")

            if 'fence_script' in parsed_params:
                fence_script_to_use = parsed_params.pop('fence_script')
                self.get_logger().info(f"Fence script path overridden by topic: {fence_script_to_use}")

            current_planning_params.update(parsed_params) # Update other planning parameters
            self.get_logger().info(f"Effective planning parameters: {current_planning_params}")
            status_msg.data = "Parameters updated from topic message."
            self.status_pub.publish(status_msg)
            
        except (ValueError, TypeError) as e:
            self.get_logger().warn(f"Could not parse JSON from trigger message: {e}. Using parameters from node initialization.")
            status_msg.data = "Could not parse JSON from trigger message. Using default parameters."
            self.status_pub.publish(status_msg)
            # Continue using initialized parameters if JSON parsing fails


        if not kml_file_path_to_use:
            self.get_logger().error("Error: KML file path is empty. Cannot proceed with planning.")
            status_msg.data = "Error: KML file path is empty. Planning aborted."
            self.status_pub.publish(status_msg)
            return # Exit callback if KML file path is missing or empty

        try:
            home_coords = self.get_home_coordinates()
            polygon = parse_kml_polygon(kml_file_path_to_use) # Use the determined KML file path
            
            # Pass all relevant parameters to generate_lawnmower_pattern
            flight_lines = generate_lawnmower_pattern(
                polygon, 
                line_spacing=current_planning_params['line_spacing'], 
                angle=current_planning_params['angle'], # Pass the possibly sentinel angle
                home_coords=home_coords, 
                optimize_angle=current_planning_params['optimize_angle'], # Pass optimize_angle flag
                buffer_distance=0.00007 # Fixed buffer, or make it a param
            )

            if not flight_lines:
                status_msg.data = "Flight line generation failed. Check polygon and parameters."
                self.status_pub.publish(status_msg)
                raise ValueError("Flight line generation failed. Check polygon and parameters.")
            
            waypoints_data = create_waypoints_from_pattern(flight_lines, home_coords)
            fence_polygon_obj = generate_geofence_polygon(
                polygon, 
                current_planning_params['fence_buffer'], 
                method='simplified_buffer', # You can make this a parameter if needed
                home_coords=home_coords
            )
            
            # --- Save Files ---
            # Use the determined output directory
            survey_mission_file = os.path.join(output_dir_to_use, 'survey_mission.waypoints')
            geofence_file = os.path.join(output_dir_to_use, 'geofence.waypoints')
            kml_output_file = os.path.join(output_dir_to_use, 'lawnmower_pattern.kml')
            mavlink_json_file = os.path.join(output_dir_to_use, 'mission.json')


            # Ensure the output directory exists
            os.makedirs(output_dir_to_use, exist_ok=True)

            save_waypoints_mission_planner(
                waypoints_data, survey_mission_file,
                current_planning_params['flight_altitude'], current_planning_params['flight_velocity'])
            
            # Also save as MAVLink JSON if desired
            save_waypoints_mavlink(
                waypoints_data, mavlink_json_file,
                current_planning_params['flight_altitude'], current_planning_params['flight_velocity']
            )

            save_geofence_waypoints(
                polygon, geofence_file,
                current_planning_params['fence_buffer'], home_coords)
            
            save_pattern_to_kml(
                flight_lines, kml_output_file,
                "Mission Pattern", home_coords, fence_polygon_obj)

            # --- Publish Summary ---
            self.publish_mission_summary(current_planning_params, waypoints_data, flight_lines, home_coords, polygon)
            
            status_msg.data = f"Success: Mission files generated in {output_dir_to_use}"
            self.status_pub.publish(status_msg)
            
            # Parse the saved mission file into mavros_msgs.msg.Waypoint objects
            mission_waypoints_for_push = self.parse_waypoint_file(survey_mission_file)
            if not mission_waypoints_for_push:
                self.get_logger().error("No waypoints found in generated mission file for push operation.")
                return

            self.push_mission(mission_waypoints_for_push) # Push mission to FCU

            # Run pymavlink_fence.py using subprocess
            pymavlink_fence_script_path = fence_script_to_use 
            
            if not os.path.exists(pymavlink_fence_script_path):
                self.get_logger().error(f"pymavlink_fence.py script not found at {pymavlink_fence_script_path}. Skipping fence upload.")
                status_msg.data = "Warning: pymavlink_fence.py script not found. Geofence not uploaded."
                self.status_pub.publish(status_msg)
            else:
                self.get_logger().info(f"Uploading geofence using {pymavlink_fence_script_path}...")
                args = ["python3", pymavlink_fence_script_path, "--fence_path", geofence_file]
                process = subprocess.Popen(args, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
                
                try:
                    stdout, stderr = process.communicate(timeout=30)
                except subprocess.TimeoutExpired:
                    process.kill()
                    stdout, stderr = process.communicate()
                    self.get_logger().error("Fence upload timed out. Killing the process.")

                if process.returncode == 0: 
                    self.get_logger().info(f"pymavlink_fence.py output: {stdout.decode()}")
                    self.get_logger().info("Geofence uploaded successfully by pymavlink_fence.py.")
                else:
                    self.get_logger().error(f"pymavlink_fence.py failed with error:\n{stderr.decode()}")
                    self.get_logger().error(f"pymavlink_fence.py output:\n{stdout.decode()}")
                
                time.sleep(2)
                #self.set_fence_parameters() # Set FCU parameters to enable the fence
                status_msg.data = "Uploaded Mission and Fence to FC"
                self.status_pub.publish(status_msg)

        except (IOError, ValueError, Exception) as e:
            error_msg = f"Planning failed: {e}"
            self.get_logger().error(error_msg)
            self.get_logger().error(traceback.format_exc())
            status_msg.data = error_msg
            self.status_pub.publish(status_msg)

    def publish_mission_summary(self, params, waypoints_data, flight_lines, home_coords, polygon):
        """Calculates and publishes the final mission summary."""
        survey_distance = sum(line.length for line in flight_lines)
        turning_distance = 0
        if len(flight_lines) > 1:
            for i in range(len(flight_lines) - 1):
                end_pt = flight_lines[i].coords[-1]
                start_pt = flight_lines[i+1].coords[0]
                turning_distance += math.sqrt((end_pt[0] - start_pt[0])**2 + (end_pt[1] - start_pt[1])**2)
                
        total_flight_dist_deg = survey_distance + turning_distance
        total_flight_dist_m = total_flight_dist_deg * 111000

        flight_time_min = (total_flight_dist_m / params['flight_velocity'] / 60) if params['flight_velocity'] > 0 else 0
        
        lat_rad = math.radians(polygon.centroid.y)
        area_m2 = polygon.area * (111000**2) * math.cos(lat_rad)

        summary = {
            "total_waypoints": len(waypoints_data),
            "flight_altitude_m": params['flight_altitude'],
            "flight_velocity_mps": params['flight_velocity'],
            "home_coordinates": home_coords,
            "survey_lines": len(flight_lines),
            "geofence_buffer_m": params['fence_buffer'],
            "survey_distance_m": survey_distance * 111000,
            "turning_distance_m": turning_distance * 111000,
            "total_flight_distance_m": total_flight_dist_m,
            "estimated_flight_time_min": flight_time_min,
            "estimated_survey_area_m2": area_m2,
            "files_created": [
                "survey_mission.waypoints",
                "geofence.waypoints",
                "lawnmower_pattern.kml",
                "mission.json" # Added MAVLink JSON file
            ]
        }

        try:
            summary_json = json.dumps(summary, indent=4)
            summary_msg = String()
            summary_msg.data = summary_json
            self.summary_pub.publish(summary_msg)
            self.get_logger().info("Published mission summary to /planning/mission_summary")
        except TypeError as e:
            self.get_logger().error(f"Could not serialize mission summary to JSON: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = LawnmowerPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node stopped cleanly.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
