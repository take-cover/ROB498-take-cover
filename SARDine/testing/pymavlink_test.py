from pymavlink import mavutil

# --- choose ONE ---
# Serial (Cube via USB)
# master = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)

# Telemetry radio
# master = mavutil.mavlink_connection('/dev/ttyUSB0', baud=57600)

# UDP (common for PX4 + MAVROS / SITL)
master = mavutil.mavlink_connection('udp:0.0.0.0:14550')

print("Waiting for heartbeat...")
master.wait_heartbeat()

print("Heartbeat received!")
print(f"System ID: {master.target_system}")
print(f"Component ID: {master.target_component}")