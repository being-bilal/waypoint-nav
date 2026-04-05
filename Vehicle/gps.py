import csv
import time

def stream_gps():

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

            gps_pkt = {
                "lat": float(row["lat"]),
                "lon": float(row["lon"])
            }

            yield gps_pkt
