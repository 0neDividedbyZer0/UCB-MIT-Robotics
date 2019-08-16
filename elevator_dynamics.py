# -*- coding: utf-8 -*-
"""
Created on Tue Aug  6 17:55:23 2019

@author: Maverick1
"""
import cvs_friction_model as cvs
import stribeck_friction_model as stribeck
import rk4
import numpy as np
import math

class elevator_dynamics:
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
    
    self.m = 10.0 * 0.454 #kg
    
    self.A_11 = -(k_v * G * G) / (k_t * R * r_p * r_p * self.m)
    
    self.B_01 = (G * mu) / (k_t * R * r_p * self.m)
    #1500, 10.0, 100.0 or 0.0 #0, 1880.0, 1410
    self.friction = stribeck.stribeck_friction_model(1000, 1880 ,2000, 1, 0.5 )#cvs.cvs_friction_model(200, 100.0, 110)
    
    print(self.B_01 * 12.0 * self.m)
    
    
  def f(self, t, x, u):
    F = self.A_11 * self.m * x[1,0] + self.B_01 * self.m * u[0,0] - self.m * 9.80
    #print(self.friction.f(x[1,0], F))
    a = (F + self.friction.f(x[1,0], F)) / self.m
    
    v = x[1,0]
    return np.asmatrix([[v],
                        [a]])  
    
  def iterate(self, t_n, x_n, u_n, n=5):
    for i in range(n - 1):
      x_n = rk4.rk4(self.f, t_n, x_n, u_n, 0.005/n)
    return rk4.rk4(self.f, t_n, x_n, u_n, 0.005/n)
    