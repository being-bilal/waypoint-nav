import time
import xsensdeviceapi as xda

# ---------------------------------------------------------
# 1. WRAPPER CLASS (Grabs Accel + Euler)
# ---------------------------------------------------------

# --- IMU ALIGNMENT CALIBRATION ---
YAW_MOUNTING_OFFSET = 0.0

class SimpleXsens:
    def __init__(self, hz=100, target_port="COM13"):
        self.control = xda.XsControl_construct()
        
        print(f"Scanning for Xsens on {target_port}...")
        ports = xda.XsScanner_scanPorts()
        
        if ports.empty():
            raise RuntimeError("No MTi devices found on any port! Check USB connection.")
        
        # Find COM13 in the list of detected devices
        self.port = None
        for p in ports:
            if p.portName().upper() == target_port.upper():
                self.port = p
                break
                
        if self.port is None:
            available = [p.portName() for p in ports]
            raise RuntimeError(f"Xsens found, but NOT on {target_port}. Available ports: {available}")
            
        print(f"Connected successfully to Xsens on {self.port.portName()} at {self.port.baudrate()} bps")
        
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
        """Extracts Accel and Euler data and aligns axes to the vehicle."""
        if self.cb.packet:
            euler = None
            accel = None
            
            if self.cb.packet.containsOrientation():
                euler = self.cb.packet.orientationEuler()
            if self.cb.packet.containsCalibratedAcceleration():
                accel = self.cb.packet.calibratedAcceleration()

            # Array fix applied here
            if euler is not None and accel is not None:
                aligned_yaw = euler.yaw() + YAW_MOUNTING_OFFSET
                aligned_yaw = (aligned_yaw + 180) % 360 - 180

                return {
                    "accel_x": accel[0], 
                    "accel_y": accel[1], 
                    "accel_z": accel[2],
                    "roll": euler.roll(),   
                    "pitch": euler.pitch(), 
                    "yaw": aligned_yaw      
                }
        return None
    
    def close(self):
        """Safely shuts down the connection."""
        self.device.removeCallbackHandler(self.cb)
        self.control.closePort(self.port.portName())
        self.control.destruct()

# ---------------------------------------------------------
# 2. DROP-IN GENERATOR
# ---------------------------------------------------------
def stream_imu():
    """Live IMU stream generator. Yields one dictionary packet at ~100Hz."""
    print("Initializing Xsens IMU...")
    # You can change "COM13" here if your port ever changes
    sensor = SimpleXsens(hz=100, target_port="COM13") 
    
    try:
        while True:
            imu_pkt = sensor.get_imu_packet()
            
            if imu_pkt is not None:
                yield imu_pkt
            
            time.sleep(0.01)
            
    finally:
        print("Closing Xsens IMU connection...")
        sensor.close()