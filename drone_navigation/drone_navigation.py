from pymavlink import mavutil
import time
import math

import matplotlib.pyplot as plt

def connect_to_drone():
    # Connect to the drone
    print("Connecting to the drone...")
    master = mavutil.mavlink_connection('127.0.0.1:14550')  # Replace with your connection string
    master.wait_heartbeat()
    print("Connection established. Heartbeat received.")
    return master
    

def insert_waypoint(waypoints, insertion_at, heading, distance=100, turn="right"):
    waypoint = waypoints[insertion_at]
    # Earth's radius in meters
    R = 6371000

    # Convert latitude and longitude from degrees to radians
    lat_rad = math.radians(waypoint["lat"])
    lon_rad = math.radians(waypoint["lon"])

    # Convert heading to radians
    heading += 90 if turn == "right" else -90
    heading_rad = math.radians(heading)

    # Calculate the new latitude and longitude in radians
    new_lat_rad = lat_rad + (distance / R) * math.cos(heading_rad)
    new_lon_rad = lon_rad + (distance / R) * math.sin(heading_rad) / math.cos(lat_rad)

    # Convert new latitude and longitude from radians back to degrees
    new_lat = math.degrees(new_lat_rad)
    new_lon = math.degrees(new_lon_rad)

    new_waypoint = {"lat": new_lat, "lon": new_lon, "alt": 10}
    waypoints.insert(insertion_at, new_waypoint)
    waypoints.insert(insertion_at+1, new_waypoint) # because its not going to the wp0
    
    print(f"new waypoint: Latitude {new_waypoint['lat']}, Longitude {new_waypoint['lon']}")
    new_wp_distance = get_distance(waypoints[insertion_at], new_waypoint)
    print(f"its {new_wp_distance}m away from the current waypoint")
    



def arm(master):            
    while True:
        # Wait until the drone is armed
        master.wait_heartbeat()
        
        msg = master.recv_match(type='HEARTBEAT', blocking=True, timeout=10)
        if msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
            print("Drone is armed.")
            break
        else:
            print("Arming the drone...")
            master.mav.command_long_send(
                master.target_system,
                master.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,  # Confirmation
                1,  # 1 to arm, 0 to disarm
                0, 0, 0, 0, 0, 0
            )            

    
def set_mode(master, mode):
    print(f"Checking current mode...")

    # Get the current mode
    while True:
        msg = master.recv_match(type='HEARTBEAT', blocking=True, timeout=5)
        if msg:
            current_mode = mavutil.mode_string_v10(msg)
            print(f"Current mode: {current_mode}")
            break

    # If the current mode matches the desired mode, no action is needed
    if current_mode == mode:
        print(f"Mode is already {mode}. No change required.")
        return

    # Get the mode ID for the desired mode
    available_modes = master.mode_mapping()
    if mode not in available_modes:
        print(f"Error: Mode {mode} is not supported by this vehicle.")
        return

    mode_id = available_modes[mode]
    print(f"Changing mode to {mode}...")

    # Send the command to set the mode
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id
    )

    # Wait for the mode to change\
    while True:  # 10-second timeout
        msg = master.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
        if msg:
            current_mode = mavutil.mode_string_v10(msg)
            if current_mode == mode:
                print(f"Mode successfully changed to {mode}.")
                return
        print("Waiting for mode to change...")
        time.sleep(1)


def takeoff(master, target_altitude):
    print(f"Taking off to {target_altitude} meters...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,  # Confirmation
        0,  # Empty parameter
        0, 0, 0, 0, 0,  # Empty parameters
        target_altitude  # Altitude in meters
    )

    # Monitor altitude until the drone reaches the target altitude
    while True:
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        if msg:
            current_altitude = msg.relative_alt / 1000.0  # Altitude in meters
            print(f"Current altitude: {current_altitude:.2f} meters")
            if current_altitude >= target_altitude - 0.5:  # Threshold for reaching altitude
                print("Target altitude reached.")
                break
        time.sleep(1)

    print("Takeoff complete.")


def land(master):
    """
    Commands the drone to land at its current location.
    """
    print("Sending land command...")

    master.mav.command_long_send(
        master.target_system,                  # Target system
        master.target_component,              # Target component
        mavutil.mavlink.MAV_CMD_NAV_LAND,     # Land command
        0,                                    # Confirmation
        0, 0, 0, 0,                           # Unused parameters
        0, 0,                                 # Latitude and longitude (0 = current location)
        0                                     # Altitude (0 = land at current altitude)
    )

    # Monitor the landing process
    while True:
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=5)
        if msg:
            current_altitude = msg.relative_alt / 1000.0  # Altitude in meters
            print(f"Current altitude: {current_altitude:.2f} meters")
            if current_altitude <= 0.1:  # Threshold to confirm landing
                print("Landing complete. Drone has touched down.")
                break
        else:
            print("Waiting for altitude updates...")
        time.sleep(1)


def get_distance(waypoint1, waypoint2):
    """
    Calculate the Haversine distance between two GPS coordinates.
    """
    R = 6371e3  # Radius of the Earth in meters
    phi1 = math.radians(waypoint1["lat"])
    phi2 = math.radians(waypoint2["lat"])
    delta_phi = math.radians(waypoint2["lat"] - waypoint1["lat"])
    delta_lambda = math.radians(waypoint2["lon"] - waypoint1["lon"])

    a = math.sin(delta_phi / 2) ** 2 + \
        math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda / 2) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    return R * c  # Distance in meters


def calculate_time(distance, ground_speed):
    if ground_speed > 0 and distance > 0:
        time_to_goal = distance / ground_speed
        return time_to_goal
    else:
        return 0  # Cannot estimate time if speed is zero


def monitor_waypoints(master, waypoints):
    goal_reached = False

    while True:
        gps_raw = master.recv_match(type='GPS_RAW_INT', blocking=True, timeout=5)
        if gps_raw:
            last_location = {"lat": (gps_raw.lat)/1e7, "lon": (gps_raw.lon)/1e7, "alt": 10}
            print(f"Current location: lon({gps_raw.lon} lat({gps_raw.lat}))")
            break

    print("Monitoring mission progress...")
    while not goal_reached:
        msg_current = master.recv_match(type='MISSION_CURRENT', blocking=True, timeout=5)
        nav_controller_output = master.recv_match(type='NAV_CONTROLLER_OUTPUT', blocking=True, timeout=5)
        vfr = master.recv_match(type='VFR_HUD', blocking=True, timeout=5)
        
        if msg_current and nav_controller_output and vfr:
            current_wp = msg_current.seq
            # total_wp = msg_current.total
            total_wp = len(waypoints)-1
            
            # #update the drone position on the matplotlib's plot
            gps_raw = master.recv_match(type='GPS_RAW_INT', blocking=True, timeout=2)
            if gps_raw:
                plot_path([last_location, {"lat": (gps_raw.lat)/1e7, "lon": (gps_raw.lon)/1e7, "alt": 10}], travelled=1)
                last_location = {"lat": (gps_raw.lat)/1e7, "lon": (gps_raw.lon)/1e7, "alt": 10}
            
            waypoint_distance = nav_controller_output.wp_dist
            # current_wp = 1
            #next waypoint(current_wp+1) to the end waypoint(total_wp-1) distance
            total_distance = 0
            for i in range(current_wp+1, total_wp-1):
                path_length = get_distance(waypoints[i], waypoints[i+1])
                total_distance += path_length
                # print(f"{i}->{i+1} path length: {path_length}")
            # print("total path", total_distance)
                
            
            total_distance += waypoint_distance
            speed = vfr.groundspeed
            remaining_time = calculate_time(total_distance, speed)
            remaining_time_to_next_wp = calculate_time(waypoint_distance, speed)
                            
            # Check if the current waypoint is the end waypoint and # Within 2 meters
            if (current_wp == total_wp) and waypoint_distance < 2:  
                print("All waypoints completed :)")
                goal_reached = True
               
            elif (0 <= current_wp <= total_wp):
                print(f"waypoint: {current_wp}/{total_wp} \t Speed: {speed:.2f}m/s \t Next_wp({waypoint_distance:.2f}m, {remaining_time_to_next_wp:.2f}s)  | last_wp({total_distance:.2f}m, {remaining_time:.2f}s)")
                
            else:
                print("No valid target waypoint available.")
        else:
            print("Waiting for mission data...")

        time.sleep(1)

    print("Mission monitoring complete.")


def send_waypoints(master, waypoints):
    print("Uploading waypoints to the drone...")

    # Clear existing mission
    master.waypoint_clear_all_send()

    # Send mission count
    master.waypoint_count_send(len(waypoints))

    # Send each waypoint
    for i, waypoint in enumerate(waypoints):        
        lat_mavlink = int(waypoint["lat"] * 1e7)  # Convert to MAVLink format
        lon_mavlink = int(waypoint["lon"] * 1e7)  # Convert to MAVLink format

        master.mav.mission_item_int_send(
            master.target_system,
            master.target_component,
            i,  # Sequence
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,  # Frame
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,  # Command
            0,  # Current (0 = not current, 1 = current)
            1,  # Auto-continue
            0, 0, 0, 0,  # Params 1-4 (Unused)
            lat_mavlink,  # Latitude
            lon_mavlink,  # Longitude
            waypoint["alt"]  # Altitude in meters
        )
            
        print(f"Uploaded waypoint {i + 1}: ({waypoint['lat']}, {waypoint['lon']}, {waypoint['alt']})")

    print("All waypoints uploaded successfully.")
    
    # Start the mission
    print("Starting the mission...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_MISSION_START,
        0,  # Confirmation
        0,  # First mission item to start from
        0, 0, 0, 0, 0, 0  # Unused params
    )
    print("Mission started.")
    monitor_waypoints(master, waypoints)

    
def plot_path(waypoints, travelled=0, show_time=0.01, label_txt="plot"):
    latitudes = [wp['lat'] for wp in waypoints]
    longitudes = [wp['lon'] for wp in waypoints]

    if travelled==1:
        # plt.plot(longitudes, latitudes, marker='o', color="green")
        # plt.scatter(longitudes[-1], latitudes[-1], color="green", label="Travelled")
        plt.plot(longitudes, latitudes, color="green", label="Travelled")
    else: 
        plt.plot(longitudes, latitudes, marker='o', label=label_txt)


    plt.title("Path")
    plt.xlabel("Longitude")
    plt.ylabel("Latitude")
    plt.grid(True)
    plt.draw()  # Update the plot dynamically
    plt.pause(show_time)  # Pause to allow rendering


if __name__ == "__main__":
    plt.ion()  # Turn on interactive mode
    
    # Define waypoints (latitude, longitude, altitude)    
    waypoints = [
        {"lat": -35.36353622, "lon": 149.16409971, "alt": 10}, 
        {"lat": -35.36353622, "lon": 149.16409971, "alt": 10}, 
        {"lat": -35.36269649, "lon": 149.16413286, "alt": 10}, 
        {"lat": -35.36178950, "lon": 149.16410239, "alt": 10}, 
        {"lat": -35.36176465, "lon": 149.16519933, "alt": 10}, 
        {"lat": -35.36230000, "lon": 149.16550000, "alt": 10},
        {"lat": -35.36177708, "lon": 149.16628103, "alt": 10}, 
        {"lat": -35.36180000, "lon": 149.16750000, "alt": 10}, 
        {"lat": -35.36210000, "lon": 149.16680000, "alt": 10}, 
        {"lat": -35.36268406, "lon": 149.16631150, "alt": 10},
        {"lat": -35.36380000, "lon": 149.16780000, "alt": 10}, 
        {"lat": -35.36420000, "lon": 149.16650000, "alt": 10}, 
        {"lat": -35.36438618, "lon": 149.16628103, "alt": 10}, 
        {"lat": -35.36450000, "lon": 149.16520000, "alt": 10}, 
        {"lat": -35.36390000, "lon": 149.16370000, "alt": 10}, 
    ]
    
    
    master = connect_to_drone()
    set_mode(master, "GUIDED")
    
    arm(master)
    takeoff(master, 10)  # Takeoff to 10 meters altitude  

    insertion_at = 10
    plot_path(waypoints, label_txt="initial path")
    send_waypoints(master, waypoints[:insertion_at])
    
    # add new waypoint in the drone heading direction
    while True: 
        vfr = master.recv_match(type='VFR_HUD', blocking=True, timeout=5)
        if vfr:
            drone_heading = vfr.heading
            print(f"drone is heading at {drone_heading}")
            
            insert_waypoint(waypoints, insertion_at, drone_heading)
            
            break
    
    set_mode(master, "GUIDED")
    plot_path(waypoints, label_txt="updated path")
    send_waypoints(master, waypoints[insertion_at:])
    
    land(master)
    
    # Keep the plot open
    plt.ioff()  # Turn off interactive mode when done
    plt.show()



    

