import time
import pymavlink.mavutil as utility
import pymavlink.dialects.v20.all as dialect
import argparse # Import the argparse module for command-line arguments

# Constants
FENCE_TOTAL = "FENCE_TOTAL".encode("utf-8")
FENCE_ACTION = "FENCE_ACTION".encode("utf-8")
PARAM_INDEX = -1

# === Load polygon fence from QGC .waypoints file ===
def parse_qgc_fence_file(path):
    """
    Parses a QGC .waypoints file to extract geofence points.

    Args:
        path (str): The file path to the QGC .waypoints file.

    Returns:
        list: A list of (latitude, longitude) tuples representing fence points.

    Raises:
        ValueError: If the file is not a valid QGC WPL 110 file.
    """
    with open(path, "r") as f:
        lines = f.readlines()

    # Validate file format
    if not lines[0].strip().startswith("QGC WPL 110"):
        raise ValueError("Not a valid QGC WPL 110 file")

    fence_points = []
    # Iterate through lines, skipping the header
    for line in lines[1:]:
        parts = line.strip().split("\t")
        if len(parts) < 12:
            continue

        # Check for FENCE_POINT (5001) or NAV_WAYPOINT (16) commands
        cmd = int(parts[3])
        if cmd in (5001, 16):
            lat = float(parts[8])
            lon = float(parts[9])
            fence_points.append((lat, lon))

    return fence_points

# --- Main script execution ---
if __name__ == "__main__":
    # === Parse command-line arguments ===
    parser = argparse.ArgumentParser(
        description="Uploads a geofence from a QGC .waypoints file to a MAVLink-enabled vehicle."
    )
    parser.add_argument(
        "--device",
        type=str,
        default="udp:127.0.0.1:14550",
        help="MAVLink connection string (e.g., udp:172.17.160.1:14550, /dev/ttyACM0)"
    )
    parser.add_argument(
        "--fence_path",
        type=str,
        required=True, # Make the fence path a required argument
        help="Path to the QGC .waypoints file containing the geofence."
    )
    args = parser.parse_args()

    # === Connect to FC ===
    # Use the device string from command-line arguments
    vehicle = utility.mavlink_connection(device=args.device)
    vehicle.wait_heartbeat()
    print("Connected to system:", vehicle.target_system, ", component:", vehicle.target_component)
    # === Set geofence-related parameters ===
    print("\n--- Setting fence parameters ---")
    fence_params = {
        'FENCE_ENABLE': 1,      # Enable fence
        'FENCE_ACTION': 1,      # RTL on breach
        'FENCE_TYPE': 4,        # Polygon fence
        'FENCE_MARGIN': 1.0,    # 1 meter margin
        'FENCE_ALT_MAX': 250.0  # Max altitude
    }

    for param_name, param_value in fence_params.items():
        print(f"Setting {param_name} = {param_value}")
        vehicle.mav.send(dialect.MAVLink_param_set_message(
            target_system=vehicle.target_system,
            target_component=vehicle.target_component,
            param_id=param_name.encode("utf-8"),
            param_value=float(param_value),
            param_type=dialect.MAV_PARAM_TYPE_REAL32
        ))

    # Confirm it is set
    confirmed = False
    start_time = time.time()
    while not confirmed and (time.time() - start_time < 5):
        msg = vehicle.recv_match(type=dialect.MAVLink_param_value_message.msgname, blocking=True)
        if msg and msg.param_id == param_name and abs(msg.param_value - float(param_value)) < 1e-2:
            print(f"{param_name} confirmed.")
            confirmed = True
        time.sleep(0.1)
    if not confirmed:
        print(f"Warning: {param_name} not confirmed within timeout.")

    # === Load fence list ===
    # Use the fence path from command-line arguments
    try:
        fence_list = parse_qgc_fence_file(args.fence_path)
    except FileNotFoundError:
        print(f"Error: The specified fence file '{args.fence_path}' was not found.")
        exit(1)
    except ValueError as e:
        print(f"Error parsing fence file: {e}")
        exit(1)

    # Ensure the polygon is closed by appending the first point if necessary
    if not fence_list:
        print("Error: No fence points found in the file. Exiting.")
        exit(1)
    if fence_list[0] != fence_list[-1]:
        fence_list.append(fence_list[0])
    print(f"Loaded {len(fence_list)} fence points.")

    # === Step 1: Get current FENCE_ACTION ===
    print("\n--- Getting original FENCE_ACTION ---")
    vehicle.mav.send(dialect.MAVLink_param_request_read_message(
        target_system=vehicle.target_system,
        target_component=vehicle.target_component,
        param_id=FENCE_ACTION,
        param_index=PARAM_INDEX
    ))

    fence_action_original = None
    while fence_action_original is None:
        msg = vehicle.recv_match(type=dialect.MAVLink_param_value_message.msgname, blocking=True)
        if msg and msg.param_id == "FENCE_ACTION":
            fence_action_original = int(msg.param_value)
            print("Original FENCE_ACTION:", fence_action_original)
            break
        time.sleep(0.1) # Small delay to avoid busy-waiting

    # === Step 2: Disable fence action ===
    print("\n--- Disabling FENCE_ACTION ---")
    while True:
        vehicle.mav.send(dialect.MAVLink_param_set_message(
            target_system=vehicle.target_system,
            target_component=vehicle.target_component,
            param_id=FENCE_ACTION,
            param_value=dialect.FENCE_ACTION_NONE,
            param_type=dialect.MAV_PARAM_TYPE_REAL32
        ))
        msg = vehicle.recv_match(type=dialect.MAVLink_param_value_message.msgname, blocking=True)
        if msg and msg.param_id == "FENCE_ACTION" and int(msg.param_value) == dialect.FENCE_ACTION_NONE:
            print("FENCE_ACTION disabled")
            break
        time.sleep(0.1)

    # === Step 3: Reset FENCE_TOTAL to 0 ===
    print("\n--- Resetting FENCE_TOTAL ---")
    while True:
        vehicle.mav.send(dialect.MAVLink_param_set_message(
            target_system=vehicle.target_system,
            target_component=vehicle.target_component,
            param_id=FENCE_TOTAL,
            param_value=0,
            param_type=dialect.MAV_PARAM_TYPE_REAL32
        ))
        msg = vehicle.recv_match(type=dialect.MAVLink_param_value_message.msgname, blocking=True)
        if msg and msg.param_id == "FENCE_TOTAL" and int(msg.param_value) == 0:
            print("FENCE_TOTAL reset to 0")
            break
        time.sleep(0.1)

    # === Step 4: Set FENCE_TOTAL to new length ===
    print("\n--- Setting new FENCE_TOTAL ---")
    while True:
        vehicle.mav.send(dialect.MAVLink_param_set_message(
            target_system=vehicle.target_system,
            target_component=vehicle.target_component,
            param_id=FENCE_TOTAL,
            param_value=len(fence_list),
            param_type=dialect.MAV_PARAM_TYPE_REAL32
        ))
        msg = vehicle.recv_match(type=dialect.MAVLink_param_value_message.msgname, blocking=True)
        if msg and msg.param_id == "FENCE_TOTAL" and int(msg.param_value) == len(fence_list):
            print(f"FENCE_TOTAL set to {len(fence_list)}")
            break
        time.sleep(0.1)

    # === Step 5: Send all fence points ===
    print("\n--- Uploading fence points ---")
    for idx, (lat, lon) in enumerate(fence_list):
        print(f"Uploading point {idx}: ({lat}, {lon})")
        vehicle.mav.send(dialect.MAVLink_fence_point_message(
            target_system=vehicle.target_system,
            target_component=vehicle.target_component,
            idx=idx,
            count=len(fence_list),
            lat=lat,
            lng=lon
        ))
        # Request confirmation for the point just sent
        vehicle.mav.send(dialect.MAVLink_fence_fetch_point_message(
            target_system=vehicle.target_system,
            target_component=vehicle.target_component,
            idx=idx
        ))

        # Wait for confirmation
        confirmed = False
        start_time = time.time()
        while not confirmed and (time.time() - start_time < 5): # Timeout after 5 seconds
            msg = vehicle.recv_match(type=dialect.MAVLink_fence_point_message.msgname, blocking=True)
            if msg and msg.idx == idx:
                if abs(msg.lat - lat) < 1e-7 and abs(msg.lng - lon) < 1e-7: # Compare floats with tolerance
                    print(f"Point {idx} confirmed.")
                    confirmed = True
                else:
                    print(f"Warning: Mismatch on point {idx}. Expected ({lat}, {lon}), Received ({msg.lat}, {msg.lng}).")
                break # Exit inner loop once message for current idx is received
            time.sleep(0.05) # Small delay
        if not confirmed:
            print(f"Error: Timed out waiting for confirmation for point {idx}.")


    # === Step 6: Restore original FENCE_ACTION ===
    print("\n--- Restoring original FENCE_ACTION ---")
    while True:
        vehicle.mav.send(dialect.MAVLink_param_set_message(
            target_system=vehicle.target_system,
            target_component=vehicle.target_component,
            param_id=FENCE_ACTION,
            param_value=fence_action_original,
            param_type=dialect.MAV_PARAM_TYPE_REAL32
        ))
        msg = vehicle.recv_match(type=dialect.MAVLink_param_value_message.msgname, blocking=True)
        if msg and msg.param_id == "FENCE_ACTION" and int(msg.param_value) == fence_action_original:
            print(f"FENCE_ACTION restored to {fence_action_original}")
            break
        time.sleep(0.1)

    print("\n✅ Fence uploaded and activated successfully.")



