# -*- coding: utf-8 -*-
"""
Created on Mon Aug  5 20:26:14 2019

@author: Maverick1
"""
import controls
import state_space_controller as ss
import scipy as sci
import numpy as np
from matplotlib import pyplot
import math
import pid_controller
from elevator_dynamics import elevator_dynamics


class elevator:
  def __init__(self):
    n = 2
    tau_stall = 0.71 * n #N/m
    I_stall = 134 * n# A
    omega_free = 18730.0 * 2.0 * math.pi / 60.0 # rad / s
    V_max = 12.0 #V
    I_free = 0.7 * n
    mu = 1.0
    G = 63.0 
    
    #resistance
    R = V_max / I_stall
    
    #voltage constant
    k_v = (V_max - I_free * R) / omega_free
    
    #torque constant
    k_t = I_stall / tau_stall
    
    #pulley_radius
    r_p = 0.0254
    
    m = 10.0 * 0.454
    
    A_11 = -(k_v * G * G) / (k_t * R * r_p * r_p * m)
    
    B_01 = (G * mu) / (k_t * R * r_p * m)
    
    A_c = np.asmatrix([[0.0, 1.0],
                       [0.0, A_11]])
    
    B_c = np.asmatrix([[0.0],
                       [B_01]])
    
    C = np.asmatrix([[1.0, 0.0]])
    
    D = np.asmatrix([[0.0]])
    
    Q_lqr = np.asmatrix([[3.0 ** 2, 0.0],
                         [0.0, 1.5 ** 2]])
    
    Q_kal = np.asmatrix([[.01 ** 2, 0.0],
                         [0.0, .01 ** 2]])
    
    R = np.asmatrix([[0.001 ** 2]])
    
    self.minU = np.asmatrix([[-V_max]])
    self.maxU = np.asmatrix([[V_max]])    
    A, B, Q_d, R_d = controls.c2d(A_c, B_c, 0.005, Q_lqr, R)
    
    A, B, Q_kalman, R_d = controls.c2d(A_c, B_c, 0.005, Q_kal, R)
    
    self.dt = 0.005
    
    self.controller = ss.state_space_controller(A, B, C, D)
    
    #pole placement
    #self.K = controls.place(A, B, (0.7, 0.1))
    
    #self.Kff = controls.feedforwards(A, B)
    
    #self.controller.L = controls.place(A.T, C.T, (-0.61939275,  0.98497246)).T
    
    #LQG
    self.K = controls.dlqr(A, B, Q_d, R_d)
    
    print(np.linalg.eig(A - B * self.K))
    
    self.Kff = controls.feedforwards(A, B, Q_d)
    
    self.controller.L = controls.dkalman(A, C, Q_kalman, R_d)
    
    print(np.linalg.eig(A.T - C.T * self.controller.L.T))
    
  def run_test(self, initial_X, goal, use_observer = True, iterations = 4000):
    self.controller.X = initial_X
    self.controller.X_hat = self.controller.X
    
    """
    delta = np.identity(2) - self.controller.A + self.controller.B * self.K
    delta = sci.linalg.inv(delta) #* self.controller.B * self.K * goal
    eps = sci.linalg.inv(self.controller.B * self.K)
    
    
    print(eps)
    print(delta)
    """
    
    if use_observer:
      self.controller.X_hat = initial_X + 0.03
    t = []
    y = []
    v = []
    y_hat = []
    v_hat = []
    u = []
    
    goal = goal / 39.3701
    for i in range(iterations):
      if not use_observer:
        self.controller.X_hat = self.controller.X
      y_hat.append(self.controller.X_hat[0,0] * 39.3701)
      v_hat.append(self.controller.X_hat[1,0] * 39.3701)
      U = self.Kff * (goal - self.controller.A * goal) + self.K * (goal - self.controller.X_hat)
      U = np.clip(U, self.minU, self.maxU)
      y.append(self.controller.X[0,0] * 39.3701)
      v.append(self.controller.X[1,0] * 39.3701)
      
      if use_observer:
        self.controller.predict_observer(U)
      self.controller.update(U)
      
      if use_observer:
        self.controller.correct_observer(U)
        
      t.append(i * self.dt)
      u.append(U[0,0])
      
    fig_size = [12, 9]
    pyplot.rcParams["figure.figsize"] = fig_size
    pyplot.subplot(2, 1, 1)
    pyplot.plot(t, y, label='height')
    pyplot.plot(t, v, label='velocity')
    print("Final State: ")
    print(self.controller.X * 39.3701)
    print("\n")
    if use_observer:
      pyplot.plot(t, y_hat, label='x_hat height')
      pyplot.plot(t, v_hat, label='x_hat velocity')
      print("Final Estimated State: ")
      print(self.controller.X_hat * 39.3701)
      print("\n")
    pyplot.legend()

    pyplot.subplot(2, 1, 2)
    pyplot.plot(t, u, label='voltage')
    pyplot.legend()
    pyplot.show()
    
  def run_pid_test(self, constants, initial_X, goal, reality = False, iterations = 4000):
    if reality:
      dynamics = elevator_dynamics()
      
    self.controller.X = initial_X / 39.3701
    controller = pid_controller.PIDController(constants[0], constants[1], constants[2], self.dt, goal[0,0])

    t = []
    y = []
    v = []
    e = []
    u = []
    
    for i in range(iterations):
      U = [controller.Update(self.controller.X[0,0] * 39.3701)]
      U = np.clip(U, self.minU, self.maxU)
      y.append(self.controller.X[0,0]* 39.3701)
      v.append(self.controller.X[1,0]* 39.3701)
      e.append((goal[0,0]-self.controller.X[0,0])* 39.3701)
      if reality:
        self.controller.update(U, dynamics.iterate(i * self.dt, self.controller.X, U))
      else:
        self.controller.update(U)
      t.append(i * self.dt)
      u.append(U[0,0])
      
    fig_size = [12, 9]
    pyplot.rcParams["figure.figsize"] = fig_size
    pyplot.subplot(2, 1, 1)
    pyplot.plot(t, y, label='height')
    pyplot.plot(t, v, label= 'velocity')
    print("Final State: ")
      
    print(self.controller.X * 39.3701)
    print("\n")
    #pyplot.plot(t, error, label='error')
    pyplot.legend()
    
    pyplot.subplot(2, 1, 2)
    pyplot.plot(t, u, label='voltage')
    pyplot.legend()
    pyplot.show()
      
def main():
  elev = elevator()
  
  elev.run_test(np.asmatrix([[0.0],[0.0]]), np.asmatrix([[60.0],[0.]]))
  #elev.run_pid_test((1, 0., 0.), np.asmatrix([[0.0],[0.0]]), np.asmatrix([[60.0],[0.]]), False, 4000)
    
        
if __name__ == '__main__':
  main()
    
    
    
    
    
    