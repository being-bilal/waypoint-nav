import csv
import time

def stream_imu():
    # Reading Dummy IMU data from CSV and yielding one packet at a time with timing
    # !! Replace with real IMU readings !!
    with open("../dummy_data/telemetry_log.csv", newline="") as f:
        reader = csv.DictReader(f)

        prev_time = None

        for row in reader:

            t = float(row["timestamp"])

            if prev_time is not None:
                dt = t - prev_time
                if dt > 0:
                    time.sleep(dt)

            prev_time = t

            imu_pkt = {
                "accel_x": float(row["accel_x"]),
                "accel_y": float(row["accel_y"]),
                "accel_z": float(row["accel_z"]),
                "gyro_x": float(row["roll_deg"]),
                "gyro_y": float(row["pitch_deg"]),
                "gyro_z": float(row["yaw_deg"])
            }

            yield imu_pkt