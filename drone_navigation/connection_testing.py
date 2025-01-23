from pymavlink import mavutil
import time

# Define the connection string for SITL (e.g., 'tcp:127.0.0.1:5760' for local SITL)
connection_string = '127.0.0.1:14550'

# Create a connection to the SITL vehicle
print(f"Connecting to vehicle on {connection_string}...")
master = mavutil.mavlink_connection(connection_string)

# Wait until the connection is established
master.wait_heartbeat()
print("Connected to vehicle")

# Request some basic information (e.g., GPS status, system status)
master.mav.request_data_stream_send(master.target_system, master.target_component,
                                    mavutil.mavlink.MAV_DATA_STREAM_ALL, 1, 1)
count = 0

# Main loop: Monitor vehicle data
try:
    while True:
        # vfr = master.recv_match(type='VFR_HUD', blocking=True, timeout=5)
        # if vfr:
        #     ground_speed = vfr.groundspeed 
        #     print(f"gps speed {ground_speed}")
            
        # # Read messages from the vehicle
        # msg = master.recv_match(blocking=True)
        # # Break the loop if you want to stop the connection
        # if count > 30:  # 60 seconds of monitoring
        #     print("Stopping the connection after 60 iterations")
        #     break
        # else:
        #     # Print the message (can be filtered to print only specific messages)
        #     print(msg, "\n")
        #     count += 1
            
        #update the drone position on the matplotlib's plot
        gps_raw = master.recv_match(type='GPS_RAW_INT', blocking=True, timeout=5)
        if gps_raw:
            print(f"lat: {(gps_raw.lon)} lon {(gps_raw.lat)}")
        
        # if msg is None:
        #     continue
        # # # If it's a GPS message, print the GPS data
        # if msg.get_type() == 'VFR_HUD':
        #     print(f"gps speed {msg.groundspeed}")


        # # If it's a GPS message, print the GPS data
        # if msg.get_type() == 'GPS_RAW_INT':
        #     print(f"GPS: Lat: {msg.lat}, Lon: {msg.lon}, Alt: {msg.alt}")
        #     print("\n\n")
        #     count += 1

        # # If it's a GPS message, print the GPS data
        # if msg.get_type() == 'HEARTBEAT':
        #     print(f"GPS: Lat: {msg.lat}, Lon: {msg.lon}, Alt: {msg.alt}")
        #     print("\n\n")
        #     count += 1

except KeyboardInterrupt:
    print("Disconnected by user.")

# Close the connection
master.close()
print("Connection closed.")
