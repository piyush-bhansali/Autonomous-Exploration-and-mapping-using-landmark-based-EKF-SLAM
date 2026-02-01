import numpy as np

class EKF:

    def __init__(self):

        # State vector: [x, y, theta] in map frame
        self.state = np.zeros(3)

        # State covariance matrix
        self.P = np.eye(3) * 0.1

        
        self.Q = np.diag([
            0.01,    # Distance variance: σ_Δd² (increases with motion)
            0.005    # Angular variance: σ_Δθ²
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

        self.R_features = np.diag([
            0.0025,   # x: 0.0025 m² (σ = 0.05m = 5cm, feature-based matching)
            0.0025,   # y: 0.0025 m² (σ = 0.05m = 5cm)
            0.0004    # theta: 0.0004 rad² (σ = 0.02 rad ≈ 1.1°)
        ])

        # Initialization flag
        self.initialized = False

        # Statistics
        self.prediction_count = 0
        self.update_count = 0
        self.feature_update_count = 0

    def initialize(self, x, y, theta):
        
        self.state = np.array([x, y, theta])
        self.initialized = True

    def predict_with_relative_motion(self, delta_d, delta_theta):
        
        if not self.initialized:
            return

        x, y, theta = self.state

        # Motion model: Circular arc approximation
        theta_mid = theta + delta_theta / 2.0

        x_pred = x + delta_d * np.cos(theta_mid)
        y_pred = y + delta_d * np.sin(theta_mid)
        theta_pred = theta + delta_theta
        theta_pred = np.arctan2(np.sin(theta_pred), np.cos(theta_pred))  # Normalize to [-π, π]

        self.state = np.array([x_pred, y_pred, theta_pred])

        # State Jacobian F_x (3×3)
        # ∂f/∂[x, y, θ]
        F_x = np.array([
            [1.0, 0.0, -delta_d * np.sin(theta_mid)],
            [0.0, 1.0,  delta_d * np.cos(theta_mid)],
            [0.0, 0.0,  1.0]
        ])

        # Control Jacobian F_u (3×2)
        # ∂f/∂[Δd, Δθ]
        F_u = np.array([
            [np.cos(theta_mid), -0.5 * delta_d * np.sin(theta_mid)],
            [np.sin(theta_mid),  0.5 * delta_d * np.cos(theta_mid)],
            [0.0,                1.0]
        ])

        motion_distance = abs(delta_d)
        motion_rotation = abs(delta_theta)

        
        Q_scaled = np.diag([
            self.Q[0, 0] * (motion_distance + 0.01),  # Distance noise (small base to avoid zero)
            self.Q[1, 1] * (motion_rotation + 0.001)  # Angular noise
        ])

        # Covariance prediction
        self.P = F_x @ self.P @ F_x.T + F_u @ Q_scaled @ F_u.T

        # Ensure symmetry (numerical stability)
        self.P = (self.P + self.P.T) / 2.0

        self.prediction_count += 1

    def update(self, x_meas, y_meas, theta_meas, measurement_type='icp'):
        
        if not self.initialized:
            raise RuntimeError("EKF not initialized")

        # Select measurement noise based on source
        if measurement_type == 'loop_closure':
            R = self.R_loop_closure
        elif measurement_type == 'icp':
            R = self.R_icp
        else:
            raise ValueError(f"Unknown measurement type: {measurement_type}. Use 'icp' or 'loop_closure'.")

        # Observation matrix (direct observation of state)
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

        # Covariance update (Joseph form for numerical stability)
        I = np.eye(3)
        I_KH = I - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ R @ K.T

        self.P = (self.P + self.P.T) / 2.0

        self.update_count += 1

    def update_with_features(self,
                            dx_meas: float,
                            dy_meas: float,
                            dtheta_meas: float,
                            num_matches: int,
                            measurement_cov: np.ndarray = None):
        """
        EKF update using feature-based motion measurement.

        This method performs an EKF update using relative motion measurements
        derived from matched geometric features (corners, lines) between
        consecutive lidar scans.

        Args:
            dx_meas: Measured x displacement (meters)
            dy_meas: Measured y displacement (meters)
            dtheta_meas: Measured rotation (radians)
            num_matches: Number of matched features (affects confidence)
            measurement_cov: Optional 3x3 measurement covariance matrix
        """
        if not self.initialized:
            raise RuntimeError("EKF not initialized")

        # Determine measurement noise
        if measurement_cov is not None:
            R = measurement_cov
        else:
            # Scale base noise by number of matches
            # More matches = more confident measurement
            R = self.R_features.copy()
            if num_matches < 5:
                # Fewer matches → less confident → higher noise
                scale_factor = 5.0 / num_matches
                R *= scale_factor

        # Observation matrix (direct observation of pose)
        H = np.eye(3)

        # Measurement vector
        z = np.array([dx_meas, dy_meas, dtheta_meas])

        # Predicted measurement (current state estimate)
        z_pred = H @ self.state

        # Innovation (measurement residual)
        innovation = z - z_pred

        # Normalize angle difference to [-π, π]
        innovation[2] = np.arctan2(np.sin(innovation[2]), np.cos(innovation[2]))

        # Innovation covariance
        S = H @ self.P @ H.T + R

        # Kalman gain
        try:
            K = self.P @ H.T @ np.linalg.inv(S)
        except np.linalg.LinAlgError:
            # Singular covariance - skip update
            return

        # State update
        self.state = self.state + K @ innovation

        # Normalize orientation to [-π, π]
        self.state[2] = np.arctan2(np.sin(self.state[2]), np.cos(self.state[2]))

        # Covariance update (Joseph form for numerical stability)
        I = np.eye(3)
        I_KH = I - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ R @ K.T

        # Ensure symmetry
        self.P = (self.P + self.P.T) / 2.0

        self.feature_update_count += 1

    def get_state(self):
        
        return {
            'x': self.state[0],
            'y': self.state[1],
            'theta': self.state[2]
        }

    def get_statistics(self):

        return {
            'initialized': self.initialized,
            'prediction_count': self.prediction_count,
            'update_count': self.update_count,
            'feature_update_count': self.feature_update_count,
            'position_uncertainty': np.sqrt(self.P[0, 0]**2 + self.P[1, 1]**2),
            'orientation_uncertainty': np.sqrt(self.P[2, 2])
        }