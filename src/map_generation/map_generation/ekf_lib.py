import numpy as np
from map_generation.utils import (
   normalize_angle
)

class EKF:
  def __init__(self):
    
    self.state = np.zeros(3)
    self.P = np.eye(3) * 0.1

    self.vx = 0.0
    self.vy = 0.0

    self.Q = np.diag([
      0.005,
      0.005,
      0.05
    ])

    self.R = np.diag([
      0.005,
      0.005,
      0.02
    ])

    self.initialized = False
    self.prediction_count = 0  # Add this
    self.update_count = 0 

  def initialize(self, x, y, theta, vx, vy):
    self.state = np.array([x, y, theta])
    self.vx = vx
    self.vy = vy
    self.initialized = True

  def predict(self, omega, ax, ay, dt):
    if not self.initialized:
      raise RuntimeError("EKF not initialized")
    
    x, y, theta = self.state

    self.vx += ax * dt
    self.vy += ay * dt
    
    x_pred = x + (self.vx * np.cos(theta) - self.vy * np.sin(theta)) * dt
    y_pred = y + (self.vx * np.sin(theta) + self.vy * np.cos(theta)) * dt
        
    theta_pred = theta + omega * dt
    theta_pred = normalize_angle(theta_pred)    

    self.state = np.array([x_pred, y_pred, theta_pred])

    F = np.array([
        [1.0, 0.0, -self.vx * np.sin(theta) * dt - self.vy * np.cos(theta) * dt],
        [0.0, 1.0,  self.vx * np.cos(theta) * dt - self.vy * np.sin(theta) * dt],
        [0.0, 0.0,  1.0]
    ])

    self.P = F @ self.P @ F.T + self.Q
    self.P = (self.P + self.P.T) / 2.0

    self.prediction_count += 1
  
  def update(self, x_meas, y_meas, theta_meas, vx_odom = None):

    if not self.initialized:
      raise RuntimeError("EKF nor initialized")

    H = np.eye(3)

    z = np.array([x_meas, y_meas, theta_meas])

    z_pred = H @ self.state
    innovation = z - z_pred
    innovation[2] = normalize_angle(innovation[2])

    S = H @ self.P @ H.T + self.R

    try:
      S_inv = np.linalg.inv(S)
    except np.linalg.LinAlgError:
      return
  
    K = self.P @ H.T @ S_inv

    self.state = self.state + K @ innovation

    self.state[2] = normalize_angle(self.state[2])

    I = np.eye(3)
    I_KH = I - K @ H
    self.P = I_KH @ self.P @ I_KH.T + K @ self.R @ K.T
    self.P = (self.P + self.P.T) / 2.0

    if vx_odom is not None:
        self.vx = vx_odom
        self.vy = 0.0
    
    self.update_count += 1

  def get_state(self):
      return {
          'x': self.state[0],
          'y': self.state[1],
          'theta': self.state[2],
          'vx': self.vx,  # Working variable
          'vy': self.vy   # Working variable
      }
    
  def get_covariance(self):
      return self.P.copy()
    
  def get_uncertainty(self):
      return {
        'sigma_x': np.sqrt(self.P[0, 0]),
        'sigma_y': np.sqrt(self.P[1, 1]),
        'sigma_theta': np.sqrt(self.P[2, 2])
      }