import time
import xsensdeviceapi as xda

# ---------------------------------------------------------
# 1. UPDATED WRAPPER CLASS (Now grabs Accel + Euler)
# ---------------------------------------------------------
class SimpleXsens:
    def __init__(self, hz=100):
        self.control = xda.XsControl_construct()
        ports = xda.XsScanner_scanPorts()
        if ports.empty():
            raise RuntimeError("No MTi device found! Check connection.")
        
        self.port = ports[0]
        self.control.openPort(self.port.portName(), self.port.baudrate())
        self.device = self.control.device(self.port.deviceId())
        
        self.device.gotoConfig()
        config = xda.XsOutputConfigurationArray()
        
        # Configure for BOTH Euler Angles and Acceleration
        config.push_back(xda.XsOutputConfiguration(xda.XDI_EulerAngles, hz))
        config.push_back(xda.XsOutputConfiguration(xda.XDI_Acceleration, hz))
        
        self.device.setOutputConfiguration(config)
        self.device.gotoMeasurement()

        class Callback(xda.XsCallback):
            def __init__(self):
                xda.XsCallback.__init__(self)
                self.packet = None
            def onLiveDataAvailable(self, dev, packet):
                self.packet = packet
        
        self.cb = Callback()
        self.device.addCallbackHandler(self.cb)

    def get_imu_packet(self):
        """Extracts Accel and Euler data into your specific dictionary format."""
        if self.cb.packet:
            euler = None
            accel = None
            
            if self.cb.packet.containsOrientation():
                euler = self.cb.packet.orientationEuler()
            if self.cb.packet.containsCalibratedAcceleration():
                accel = self.cb.packet.calibratedAcceleration()

            # Ensure we have both before yielding to avoid NoneType errors
            if euler and accel:
                return {
                    "accel_x": accel[0],
                    "accel_y": accel[1],
                    "accel_z": accel[2],
                    "gyro_x": euler.roll(),   # Mapped to match your dummy CSV logic
                    "gyro_y": euler.pitch(),  # Mapped to match your dummy CSV logic
                    "gyro_z": euler.yaw()     # Mapped to match your dummy CSV logic
                }
        return None

    def close(self):
        """Safely shuts down the connection."""
        self.device.removeCallbackHandler(self.cb)
        self.control.closePort(self.port.portName())
        self.control.destruct()

# ---------------------------------------------------------
# 2. YOUR NEW DROP-IN GENERATOR
# ---------------------------------------------------------
def stream_imu():
    """
    Live IMU stream generator. Replaces the dummy CSV reader.
    Yields one dictionary packet at ~100Hz.
    """
    print("Initializing Xsens IMU...")
    sensor = SimpleXsens(hz=100)
    
    try:
        while True:
            # Grab the latest packet from the mailbox
            imu_pkt = sensor.get_imu_packet()
            
            if imu_pkt is not None:
                yield imu_pkt
            
            # Sleep for exactly the rate of the sensor (100Hz = 0.01s)
            # This prevents the generator from yielding the identical packet twice
            time.sleep(0.01)
            
    finally:
        # The 'finally' block is crucial here. 
        # If your main code breaks out of the loop consuming this generator 
        # (e.g., stopping the script), Python will automatically run this block
        # to safely close the hardware port.
        print("Closing Xsens IMU connection...")
        sensor.close()