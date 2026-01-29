import numpy as np

class EKF:

    def __init__(self):

        # State vector: [x, y, theta]
        self.state = np.zeros(3)

        # State covariance matrix
        self.P = np.eye(3) * 0.1

        self.vx = 0.0
        self.vy = 0.0

    
        self.R_odom = np.diag([
            0.005,   # x position variance: 0.005 m² (σ = 0.071m ≈ 7.1cm)
            0.005,   # y position variance: 0.005 m² (σ = 0.071m ≈ 7.1cm)
            0.02     # theta variance: 0.02 rad² (σ = 0.141 rad ≈ 8.1°)
        ])

        self.R_icp = np.diag([
            0.0001,   # x: 0.0001 m² (σ = 0.01m = 1cm, ICP is accurate)
            0.0001,   # y: 0.0001 m² (σ = 0.01m = 1cm)
            0.0001    # theta: 0.0001 rad² (σ = 0.01 rad ≈ 0.57°)
        ])

        self.R_loop_closure = np.diag([
            0.0025,   # x: 0.0025 m² (σ = 0.05m = 5cm, less certain than ICP)
            0.0025,   # y: 0.0025 m² (σ = 0.05m = 5cm)
            0.0001    # theta: 0.0001 rad² (σ = 0.01 rad ≈ 0.57°)
        ])

        self.Q_imu = np.diag([
            0.0001,   
            0.0001,   
            0.001     
        ])

        self.dt = 0.005  # 200 Hz

        # Initialization flag
        self.initialized = False

        # Statistics
        self.imu_prediction_count = 0
        self.update_count = 0
        self.consecutive_predictions_without_update = 0  # Drift watchdog

        # Ground truth tracking
        self.ground_truth = None
        self.error_history = []  # List of {'timestamp': t, 'pos_error': e_pos, 'orient_error': e_theta, 'ekf_state': [x,y,theta], 'gt_state': [x,y,theta]}
        self.current_errors = {'position_error': 0.0, 'orientation_error': 0.0} 

    def initialize(self, x, y, theta, vx, vy):

        self.state = np.array([x, y, theta])
        self.vx = vx
        self.vy = vy
        self.initialized = True

    def predict_imu(self, omega):

        if not self.initialized:
            return

        x, y, theta = self.state

        dx = self.vx * np.cos(theta) * self.dt
        dy = self.vx * np.sin(theta) * self.dt

        dtheta = omega * self.dt

        x_pred = x + dx
        y_pred = y + dy
        theta_pred = theta + dtheta
        theta_pred = np.arctan2(np.sin(theta_pred), np.cos(theta_pred))  

        self.state = np.array([x_pred, y_pred, theta_pred])

        # Jacobian of motion model
        F = np.array([
            [1.0, 0.0, -self.vx * np.sin(theta) * self.dt],
            [0.0, 1.0,  self.vx * np.cos(theta) * self.dt],
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
        
        if not self.initialized:
            raise RuntimeError("EKF not initialized")

        # Select measurement noise based on source
        if measurement_type == 'loop_closure':
            R = self.R_loop_closure
        elif measurement_type == 'icp':
            R = self.R_icp
        else:
            R = self.R_odom

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
            return

        K = self.P @ H.T @ S_inv

        # State update
        self.state = self.state + K @ innovation
        self.state[2] = np.arctan2(np.sin(self.state[2]), np.cos(self.state[2]))

        I = np.eye(3)
        I_KH = I - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ R @ K.T

        self.P = (self.P + self.P.T) / 2.0

        if vx_odom is not None:
            self.vx = vx_odom
            self.vy = 0.0

        self.update_count += 1
        self.consecutive_predictions_without_update = 0

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
        self.consecutive_predictions_without_update = 0

    def set_measurement_noise(self, sigma_x, sigma_y, sigma_theta, measurement_type='odom'):
        
        R = np.diag([sigma_x**2, sigma_y**2, sigma_theta**2])
        if measurement_type == 'icp':
            self.R_icp = R
        else:
            self.R_odom = R

    def is_converged(self, pos_threshold=0.05, orient_threshold=0.05):
        
        uncertainty = self.get_uncertainty()
        pos_uncertain = np.sqrt(uncertainty['sigma_x']**2 + uncertainty['sigma_y']**2)
        return pos_uncertain < pos_threshold and uncertainty['sigma_theta'] < orient_threshold