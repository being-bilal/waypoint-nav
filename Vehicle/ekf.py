import numpy as np

class GPS_IMU_EKF:
    def __init__(self, start_x, start_y, start_yaw_rad):
        """
        Initializes the EKF.
        X state vector: [x_position, y_position, yaw_heading_radians]
        """
        # 1. Initial State [x, y, yaw]
        self.X = np.array([start_x, start_y, start_yaw_rad], dtype=float)
        
        # 2. Initial Covariance Matrix (P) - our confidence in the initial state
        self.P = np.eye(3) * 1.0  
        
        # 3. Process Noise (Q) - how much we trust the IMU/Kinematic model
        # Tune these: lower = trust IMU more, higher = trust IMU less
        self.Q = np.array([
            [0.1, 0,   0],       # x noise
            [0,   0.1, 0],       # y noise
            [0,   0,   0.05]     # yaw noise
        ])
        
        # 4. Measurement Noise (R) - how much we trust the GPS
        # Tune these: lower = trust GPS more, higher = trust GPS less
        self.R = np.array([
            [2.0, 0],            # GPS x noise (variance in meters)
            [0,   2.0]           # GPS y noise (variance in meters)
        ])
        
        # Measurement mapping matrix (H) - maps state to measurements [x, y]
        self.H = np.array([
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0]
        ])
        
        # Identity matrix for updates
        self.I = np.eye(3)

    def predict(self, velocity, yaw_rate_rad, dt):
        """
        Time Update Step (runs constantly with IMU data).
        Uses forward velocity and yaw rate to predict the next position.
        """
        x, y, yaw = self.X[0], self.X[1], self.X[2]
        
        # 1. Predict State using non-linear kinematic model
        new_x = x + velocity * np.cos(yaw) * dt
        new_y = y + velocity * np.sin(yaw) * dt
        new_yaw = yaw + yaw_rate_rad * dt
        
        # Normalize yaw to [-pi, pi]
        new_yaw = (new_yaw + np.pi) % (2 * np.pi) - np.pi
        self.X = np.array([new_x, new_y, new_yaw])
        
        # 2. Calculate Jacobian of the state transition function (F)
        F = np.array([
            [1.0, 0.0, -velocity * np.sin(yaw) * dt],
            [0.0, 1.0,  velocity * np.cos(yaw) * dt],
            [0.0, 0.0,  1.0]
        ])
        
        # 3. Predict Covariance (P = F*P*F^T + Q)
        self.P = F @ self.P @ F.T + self.Q

    def update(self, gps_x, gps_y):
        """
        Measurement Update Step (runs only when fresh GPS arrives).
        Corrects the predicted state using actual GPS readings.
        """
        # 1. Measurement vector
        Z = np.array([gps_x, gps_y])
        
        # 2. Innovation (difference between measurement and prediction)
        Y = Z - (self.H @ self.X)
        
        # 3. Innovation Covariance (S = H*P*H^T + R)
        S = self.H @ self.P @ self.H.T + self.R
        
        # 4. Kalman Gain (K = P*H^T*S^-1)
        K = self.P @ self.H.T @ np.linalg.inv(S)
        
        # 5. Update State (X = X + K*Y)
        self.X = self.X + K @ Y
        self.X[2] = (self.X[2] + np.pi) % (2 * np.pi) - np.pi # Normalize yaw
        
        # 6. Update Covariance (P = (I - K*H)*P)
        self.P = (self.I - K @ self.H) @ self.P

    def get_state(self):
        """Returns [x, y, yaw_degrees]"""
        return self.X[0], self.X[1], np.degrees(self.X[2])