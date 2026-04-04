import csv
import time

def stream_imu():
    # Reading Dummy IMU data from CSV and yielding one packet at a time with timing
    # !! Replace with real IMU readings !!
    with open("../dummy_data/imu_data.csv", newline="") as f:
        reader = csv.DictReader(f)

        prev_time = None

        for row in reader:

            t = float(row["t_s"])

            if prev_time is not None:
                dt = t - prev_time
                if dt > 0:
                    time.sleep(dt)

            prev_time = t

            imu_pkt = {
                "accel_x": float(row["accel_x_ms2"]),
                "accel_y": float(row["accel_y_ms2"]),
                "accel_z": float(row["accel_z_ms2"]),
                "gyro_x": float(row["gyro_x_degs"]),
                "gyro_y": float(row["gyro_y_degs"]),
                "gyro_z": float(row["gyro_z_degs"])
            }

            yield imu_pkt