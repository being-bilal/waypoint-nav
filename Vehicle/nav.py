from imu import stream_imu
from gps import stream_gps
from magnetometer import stream_magnetometer


def transmitData():

    imu_stream = stream_imu()
    gps_stream = stream_gps()
    mag_stream = stream_magnetometer()

    while True:

        imu_data = next(imu_stream)
        gps_data = next(gps_stream)
        mag_data = next(mag_stream)

        packet = {
            "imu": {
                "accel_x": float(imu_data.get("accel_x", 0.0)),
                "accel_y": float(imu_data.get("accel_y", 0.0)),
                "accel_z": float(imu_data.get("accel_z", 0.0)),
                "gyro_x":  float(imu_data.get("gyro_x", 0.0)),
                "gyro_y":  float(imu_data.get("gyro_y", 0.0)),
                "gyro_z":  float(imu_data.get("gyro_z", 0.0))
            },
            "gps": {
                "lat": float(gps_data.get("lat", gps_data.get("latitude", 0.0))),
                "lon": float(gps_data.get("lon", gps_data.get("longitude", 0.0)))
            },
            "mag": {
                "heading_deg": float(mag_data.get("raw_heading_deg", 0.0))
            }
        }

        yield packet