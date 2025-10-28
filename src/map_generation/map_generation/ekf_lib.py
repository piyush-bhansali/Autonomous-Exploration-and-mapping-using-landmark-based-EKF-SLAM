"""
Extended Kalman Filter for 2D Robot Pose Estimation

State vector: [x, y, theta]
- Prediction: Using IMU angular velocity (high frequency)
- Update: Using odometry measurements (lower frequency)
"""

import numpy as np
from map_generation.utils import normalize_angle
import time


class EKF:
    
    def __init__(self):
        
        # State vector: [x, y, theta]
        self.state = np.zeros(3)

        # State covariance matrix
        self.P = np.eye(3) * 0.1

        # Velocity estimates (not part of state, but used for prediction)
        self.vx = 0.0
        self.vy = 0.0

        # Process noise covariance (tuned for TurtleBot3)
        self.Q = np.diag([
            0.005,  # x position process noise
            0.005,  # y position process noise
            0.05    # theta process noise (higher due to gyro drift)
        ])

        # Measurement noise covariance (from odometry)
        self.R = np.diag([
            0.005,  # x measurement noise
            0.005,  # y measurement noise
            0.02    # theta measurement noise
        ])

        # IMU-only prediction noise (for predict_imu with dead reckoning)
        self.Q_imu = np.diag([
            0.001,  # Small position uncertainty (from velocity dead reckoning)
            0.001,  # Small position uncertainty (from velocity dead reckoning)
            0.01    # Orientation uncertainty from gyroscope
        ])

        # Initialization flag
        self.initialized = False

        # Statistics
        self.prediction_count = 0
        self.imu_prediction_count = 0
        self.update_count = 0

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

    def predict(self, omega, ax, ay, dt):
        
        if not self.initialized:
            raise RuntimeError("EKF not initialized")

        x, y, theta = self.state

        # Update velocity estimates from acceleration
        self.vx += ax * dt
        self.vy += ay * dt

        # Predict new state using motion model
        x_pred = x + (self.vx * np.cos(theta) - self.vy * np.sin(theta)) * dt
        y_pred = y + (self.vx * np.sin(theta) + self.vy * np.cos(theta)) * dt
        theta_pred = theta + omega * dt
        theta_pred = normalize_angle(theta_pred)

        self.state = np.array([x_pred, y_pred, theta_pred])

        # Compute Jacobian of motion model
        F = np.array([
            [1.0, 0.0, -self.vx * np.sin(theta) * dt - self.vy * np.cos(theta) * dt],
            [0.0, 1.0,  self.vx * np.cos(theta) * dt - self.vy * np.sin(theta) * dt],
            [0.0, 0.0,  1.0]
        ])

        # Update covariance
        self.P = F @ self.P @ F.T + self.Q

        # Ensure symmetry (numerical stability)
        self.P = (self.P + self.P.T) / 2.0

        self.prediction_count += 1
        self.last_prediction_time = time.time()

    def predict_imu(self, omega, dt=None):
        
        if not self.initialized:
            return  # Silently return if not initialized yet

        # Auto-calculate dt if not provided
        if dt is None:
            current_time = time.time()
            if self.last_prediction_time is not None:
                dt = current_time - self.last_prediction_time
                # Sanity check: IMU typically runs at 200Hz (dt ~ 0.005s)
                # Reject unrealistic dt values
                if dt > 0.1 or dt <= 0:
                    dt = 0.005  # Default to 200 Hz
            else:
                dt = 0.005  # Default to 200 Hz
            self.last_prediction_time = current_time

        x, y, theta = self.state

        # Predict position using last known velocity (from odometry)
        # This provides dead reckoning between odometry updates
        dx = self.vx * np.cos(theta) * dt
        dy = self.vx * np.sin(theta) * dt

        # Update orientation from gyroscope
        dtheta = omega * dt

        # Apply predictions
        x_pred = x + dx
        y_pred = y + dy
        theta_pred = normalize_angle(theta + dtheta)

        self.state = np.array([x_pred, y_pred, theta_pred])

        # Jacobian for dead reckoning + gyro
        # Note: vx is treated as constant (not part of state)
        F = np.array([
            [1.0, 0.0, -self.vx * np.sin(theta) * dt],
            [0.0, 1.0,  self.vx * np.cos(theta) * dt],
            [0.0, 0.0,  1.0]
        ])

        # Update covariance with IMU-specific process noise
        self.P = F @ self.P @ F.T + self.Q_imu

        # Ensure symmetry
        self.P = (self.P + self.P.T) / 2.0

        self.imu_prediction_count += 1

    def update(self, x_meas, y_meas, theta_meas, vx_odom=None):
        """
        Update (correction) step using odometry measurements

        Args:
            x_meas: Measured x position from odometry (m)
            y_meas: Measured y position from odometry (m)
            theta_meas: Measured orientation from odometry (rad)
            vx_odom: Linear velocity from odometry (m/s), optional
        """
        if not self.initialized:
            raise RuntimeError("EKF not initialized")

        # Measurement matrix (direct observation of state)
        H = np.eye(3)

        # Measurement vector
        z = np.array([x_meas, y_meas, theta_meas])

        # Innovation (measurement residual)
        z_pred = H @ self.state
        innovation = z - z_pred
        innovation[2] = normalize_angle(innovation[2])  # Wrap angle difference

        # Innovation covariance
        S = H @ self.P @ H.T + self.R

        # Kalman gain
        try:
            S_inv = np.linalg.inv(S)
        except np.linalg.LinAlgError:
            # Singular matrix, skip update
            return

        K = self.P @ H.T @ S_inv

        # State update
        self.state = self.state + K @ innovation
        self.state[2] = normalize_angle(self.state[2])  # Normalize theta

        # Covariance update (Joseph form for numerical stability)
        I = np.eye(3)
        I_KH = I - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ self.R @ K.T

        # Ensure symmetry
        self.P = (self.P + self.P.T) / 2.0

        # Update velocity estimate from odometry
        if vx_odom is not None:
            self.vx = vx_odom
            self.vy = 0.0  # Differential drive: no lateral velocity

        self.update_count += 1
        self.last_update_time = time.time()

    def get_state(self):
        """
        Get current state estimate

        Returns:
            dict: State dictionary with x, y, theta, vx, vy
        """
        return {
            'x': self.state[0],
            'y': self.state[1],
            'theta': self.state[2],
            'vx': self.vx,
            'vy': self.vy
        }

    def get_covariance(self):
        """
        Get current state covariance matrix

        Returns:
            np.ndarray: 3x3 covariance matrix
        """
        return self.P.copy()

    def get_uncertainty(self):
        """
        Get current state uncertainty (standard deviations)

        Returns:
            dict: Standard deviations for x, y, theta
        """
        return {
            'sigma_x': np.sqrt(self.P[0, 0]),
            'sigma_y': np.sqrt(self.P[1, 1]),
            'sigma_theta': np.sqrt(self.P[2, 2])
        }

    def get_statistics(self):
        """
        Get filter statistics

        Returns:
            dict: Statistics including prediction/update counts
        """
        return {
            'initialized': self.initialized,
            'prediction_count': self.prediction_count,
            'imu_prediction_count': self.imu_prediction_count,
            'update_count': self.update_count,
            'total_predictions': self.prediction_count + self.imu_prediction_count,
            'position_uncertainty': np.sqrt(self.P[0, 0]**2 + self.P[1, 1]**2),
            'orientation_uncertainty': np.sqrt(self.P[2, 2])
        }

    def reset(self):
        """Reset the filter to uninitialized state"""
        self.state = np.zeros(3)
        self.P = np.eye(3) * 0.1
        self.vx = 0.0
        self.vy = 0.0
        self.initialized = False
        self.prediction_count = 0
        self.imu_prediction_count = 0
        self.update_count = 0
        self.last_prediction_time = None
        self.last_update_time = None

    def set_process_noise(self, sigma_x, sigma_y, sigma_theta):
        """
        Set process noise parameters

        Args:
            sigma_x: Process noise std dev for x (m)
            sigma_y: Process noise std dev for y (m)
            sigma_theta: Process noise std dev for theta (rad)
        """
        self.Q = np.diag([sigma_x**2, sigma_y**2, sigma_theta**2])

    def set_measurement_noise(self, sigma_x, sigma_y, sigma_theta):
        """
        Set measurement noise parameters

        Args:
            sigma_x: Measurement noise std dev for x (m)
            sigma_y: Measurement noise std dev for y (m)
            sigma_theta: Measurement noise std dev for theta (rad)
        """
        self.R = np.diag([sigma_x**2, sigma_y**2, sigma_theta**2])

    def is_converged(self, pos_threshold=0.05, orient_threshold=0.05):
        """
        Check if filter has converged (low uncertainty)

        Args:
            pos_threshold: Position uncertainty threshold (m)
            orient_threshold: Orientation uncertainty threshold (rad)

        Returns:
            bool: True if converged
        """
        uncertainty = self.get_uncertainty()
        pos_uncertain = np.sqrt(uncertainty['sigma_x']**2 + uncertainty['sigma_y']**2)
        return pos_uncertain < pos_threshold and uncertainty['sigma_theta'] < orient_threshold