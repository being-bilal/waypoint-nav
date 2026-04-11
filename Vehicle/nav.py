from accelerometer_data import Accelerometer


def transmitData():
    accel = Accelerometer()

    try:
        while True:
            accel_data = accel.get()

            packet = {
                "accel": {
                    "ax": accel_data["ax"],
                    "ay": accel_data["ay"],
                    "az": accel_data["az"],
                },
            }
            

            yield packet

    finally:
        accel.stop()