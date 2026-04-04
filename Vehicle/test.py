"""
Reads IMU + GPS data from Pixhawk over USB and prints it.
"""

from pymavlink import mavutil
import math

master = mavutil.mavlink_connection("/dev/tty.usbmodem11301", baud=115200)

print("Waiting for heartbeat...")
master.wait_heartbeat()
print("Connected!\n")

# Stream attitude at 50 Hz
master.mav.request_data_stream_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_EXTRA1, 50, 1)

# Stream raw IMU at 50 Hz
master.mav.request_data_stream_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_RAW_SENSORS, 50, 1)

# Stream GPS at 2 Hz
master.mav.request_data_stream_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_POSITION, 2, 1)

data = {"roll": 0, "pitch": 0, "yaw": 0,
        "ax": 0, "ay": 0, "az": 0,
        "lat": 0, "lon": 0, "fix": 0, "sats": 0}

while True:
    msg = master.recv_match(blocking=True)
    if not msg:
        continue

    t = msg.get_type()

    if t == "ATTITUDE":
        data["roll"]  = math.degrees(msg.roll)
        data["pitch"] = math.degrees(msg.pitch)
        data["yaw"]   = math.degrees(msg.yaw)

    elif t == "RAW_IMU":
        # RAW_IMU accel is in milli-g — convert to m/s²
        data["ax"] = msg.xacc * 9.81 / 1000
        data["ay"] = msg.yacc * 9.81 / 1000
        data["az"] = msg.zacc * 9.81 / 1000

    elif t == "GPS_RAW_INT":
        # lat/lon are in 1e-7 degrees
        data["lat"]  = msg.lat / 1e7
        data["lon"]  = msg.lon / 1e7
        data["fix"]  = msg.fix_type   # 0=no fix, 2=2D, 3=3D
        data["sats"] = msg.satellites_visible

    print(
        f"\rATTITUDE  roll={data['roll']:>+7.2f}°  pitch={data['pitch']:>+7.2f}°  yaw={data['yaw']:>+8.2f}°  |  "
        f"ACCEL  x={data['ax']:>+6.2f}  y={data['ay']:>+6.2f}  z={data['az']:>+6.2f} m/s²  |  "
        f"GPS  lat={data['lat']:.6f}  lon={data['lon']:.6f}  fix={data['fix']}  sats={data['sats']}",
        end="", flush=True
    )