import csv
import time

def stream_magnetometer():

    with open("../dummy_data/magnetometer_data.csv", newline="") as f:
        reader = csv.DictReader(f)

        prev_time = None

        for row in reader:

            t = float(row["t_s"])

            if prev_time is not None:
                dt = t - prev_time
                if dt > 0:
                    time.sleep(dt)

            prev_time = t

            mag_pkt = {
                "raw_heading_deg": float(row["raw_heading_deg"])
            }

            yield mag_pkt