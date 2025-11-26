import numpy as np
import time


class EKF:
    
    def __init__(self):
        
        # State vector: [x, y, theta]
        self.state = np.zeros(3)

        # State covariance matrix
        self.P = np.eye(3) * 0.1

        self.vx = 0.0
        self.vy = 0.0

        # Measurement noise covariance (from odometry)
        # Tuned for Gazebo simulation - odometry is quite accurate
        self.R_odom = np.diag([
            0.01,   # x: 10 cm std (odometry position accuracy)
            0.01,   # y: 10 cm std
            0.02    # theta: 0.14 rad (~8°) - reduced from 0.05 for better orientation tracking
        ])

        # Measurement noise for ICP corrections (higher confidence)
        self.R_icp = np.diag([
            0.005,  # x: 5 mm std (ICP is very accurate)
            0.005,  # y: 5 mm std
            0.01    # theta: 0.1 rad (~6°)
        ])

        
        self.Q_imu = np.diag([
            0.01,   
            0.01,   
            0.02   
        ])

        # Initialization flag
        self.initialized = False

        # Statistics
        self.imu_prediction_count = 0
        self.update_count = 0
        self.consecutive_predictions_without_update = 0  # Drift watchdog

        # Timing for adaptive dt calculation
        self.last_prediction_time = None
        self.last_update_time = None 

    def initialize(self, x, y, theta, vx, vy):
        
        self.state = np.array([x, y, theta])
        self.vx = vx
        self.vy = vy
        self.initialized = True
        self.last_prediction_time = time.time()
        self.last_update_time = time.time()

    def predict_imu(self, omega, dt=None):
        
        if not self.initialized:
            return  # Silently return if not initialized yet

        # Auto-calculate dt if not provided
        if dt is None:
            current_time = time.time()
            if self.last_prediction_time is not None:
                dt = current_time - self.last_prediction_time
                if dt > 0.1 or dt <= 0:
                    dt = 0.005  # Default to 200 Hz
            else:
                dt = 0.005  # Default to 200 Hz
            self.last_prediction_time = current_time

        x, y, theta = self.state

        dx = self.vx * np.cos(theta) * dt
        dy = self.vx * np.sin(theta) * dt

        # Update orientation from gyroscope
        dtheta = omega * dt

        # Apply predictions
        x_pred = x + dx
        y_pred = y + dy
        theta_pred = theta + dtheta
        theta_pred = np.arctan2(np.sin(theta_pred), np.cos(theta_pred))  # Normalize to [-π, π]

        self.state = np.array([x_pred, y_pred, theta_pred])

        F = np.array([
            [1.0, 0.0, -self.vx * np.sin(theta) * dt],
            [0.0, 1.0,  self.vx * np.cos(theta) * dt],
            [0.0, 0.0,  1.0]
        ])

        self.P = F @ self.P @ F.T + self.Q_imu

        # Ensure symmetry
        self.P = (self.P + self.P.T) / 2.0

        self.imu_prediction_count += 1
        self.consecutive_predictions_without_update += 1

        if self.consecutive_predictions_without_update > 100:  # ~0.5s at 200Hz
            import warnings
            warnings.warn(f"EKF: {self.consecutive_predictions_without_update} IMU predictions without odometry update - potential drift!", stacklevel=2)

    def update(self, x_meas, y_meas, theta_meas, vx_odom=None, measurement_type='odom'):
        """
        Update EKF state with measurement.

        Args:
            x_meas: Measured x position
            y_meas: Measured y position
            theta_meas: Measured orientation
            vx_odom: Linear velocity from odometry (optional)
            measurement_type: 'odom' or 'icp' - determines measurement noise
        """
        if not self.initialized:
            raise RuntimeError("EKF not initialized")

        # Select measurement noise based on source
        if measurement_type == 'icp':
            R = self.R_icp  # High confidence (5mm position, 0.1 rad orientation)
        else:
            R = self.R_odom  # Normal odometry confidence (10cm position, 0.14 rad orientation)

        H = np.eye(3)

        # Measurement vector
        z = np.array([x_meas, y_meas, theta_meas])

        # Innovation (measurement residual)
        z_pred = H @ self.state
        innovation = z - z_pred
        innovation[2] = np.arctan2(np.sin(innovation[2]), np.cos(innovation[2]))  # Wrap angle difference to [-π, π]

        # Innovation covariance
        S = H @ self.P @ H.T + R

        # Kalman gain
        try:
            S_inv = np.linalg.inv(S)
        except np.linalg.LinAlgError:
            # Singular matrix, skip update
            return

        K = self.P @ H.T @ S_inv

        # State update
        self.state = self.state + K @ innovation
        self.state[2] = np.arctan2(np.sin(self.state[2]), np.cos(self.state[2])) 


        I = np.eye(3)
        I_KH = I - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ R @ K.T  # Use measurement-specific R

        # Ensure symmetry
        self.P = (self.P + self.P.T) / 2.0

        # Update velocity estimate from odometry
        if vx_odom is not None:
            self.vx = vx_odom
            self.vy = 0.0  

        self.update_count += 1
        self.consecutive_predictions_without_update = 0  # Reset drift watchdog
        self.last_update_time = time.time()

    def get_state(self):
        
        return {
            'x': self.state[0],
            'y': self.state[1],
            'theta': self.state[2],
            'vx': self.vx,
            'vy': self.vy
        }

    def get_covariance(self):
        
        return self.P.copy()

    def get_uncertainty(self):
        
        return {
            'sigma_x': np.sqrt(self.P[0, 0]),
            'sigma_y': np.sqrt(self.P[1, 1]),
            'sigma_theta': np.sqrt(self.P[2, 2])
        }

    def get_statistics(self):

        return {
            'initialized': self.initialized,
            'imu_prediction_count': self.imu_prediction_count,
            'update_count': self.update_count,
            'total_predictions': self.imu_prediction_count,
            'position_uncertainty': np.sqrt(self.P[0, 0]**2 + self.P[1, 1]**2),
            'orientation_uncertainty': np.sqrt(self.P[2, 2])
        }

    def reset(self):
        
        self.state = np.zeros(3)
        self.P = np.eye(3) * 0.1
        self.vx = 0.0
        self.vy = 0.0
        self.initialized = False
        self.imu_prediction_count = 0
        self.update_count = 0
        self.last_prediction_time = None
        self.last_update_time = None
        self.consecutive_predictions_without_update = 0

    def set_measurement_noise(self, sigma_x, sigma_y, sigma_theta, measurement_type='odom'):
        """
        Set measurement noise covariance.

        Args:
            sigma_x: Standard deviation of x measurement (m)
            sigma_y: Standard deviation of y measurement (m)
            sigma_theta: Standard deviation of theta measurement (rad)
            measurement_type: 'odom' or 'icp'
        """
        R = np.diag([sigma_x**2, sigma_y**2, sigma_theta**2])
        if measurement_type == 'icp':
            self.R_icp = R
        else:
            self.R_odom = R

    def is_converged(self, pos_threshold=0.05, orient_threshold=0.05):
        
        uncertainty = self.get_uncertainty()
        pos_uncertain = np.sqrt(uncertainty['sigma_x']**2 + uncertainty['sigma_y']**2)
        return pos_uncertain < pos_threshold and uncertainty['sigma_theta'] < orient_threshold