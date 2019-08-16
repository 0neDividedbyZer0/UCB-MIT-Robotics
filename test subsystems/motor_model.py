# -*- coding: utf-8 -*-
"""
Created on Sun Aug  4 16:47:51 2019

@author: Maverick Zhang
"""


"""
This class represents a model of a DC electric motor. 

Explanation of Physics/Math:
  A DC motor can be understood as an open circuit with 
  a resistor due to the resistance in the copper windings 
  and a voltage source generated from the spinning armature
  (a back emf is created due to a spinning magnetic field).
  Thus, the equation for the circuit of a DC motor can be 
  modelled as 
      u = V + IR
  where u is the voltage you have sent to the motor leads,
  V is the back emf, and IR is the voltage from the resistor 
  (copper windings). In motor usage, you change the value of
  u and that create different effects in the spinning of the
  motor. To model a DC motor properly, you need to specify 
  a few more variables which most electric motor manufacturers
  will publish in a motor curve. The explanation for the model
  will be below. (This model is not perfect, little nonlinear
  effects will show up because the resistor and generator are 
  not created directly after each other, but this equation
  will work extremely well).
"""

class motor_model:
  """
  Please put all variables in metric. Efficiency is given as
  a percentage. We can take the original equation 
      u = V + IR
  and modify it to be in terms of torques and angular velocity:
      mu * u = k_v * omega + k_t * tau * R
  where mu is efficiency, k_v is known as the voltage constant,
  k_t is known as the torque constant, and tau is torque.
  Further modification of the equation to include gear_ratios
  and different numbers of motors, the equation becomes
      mu * u = k_v * G * omega + (k_t * tau * R) / (n * G)
  where G is the gear ratio and n is the number of motors.
  (We use the convention where a gear ratio intended to
  increase output torque is considered > 1)
  """
  def __init__(self, stall_torque, stall_current, free_speed,
               max_voltage, free_current = 0.0, efficiency = 1.0,
               num_motors = 1, gear_ratio = 1.0):
    self.tau_stall = stall_torque
    self.I_stall = stall_current
    self.omega_free = free_speed
    self.V_max = max_voltage
    self.I_free = free_current
    self.mu = efficiency
    self.n = num_motors
    self.G = gear_ratio 
    
    #resistance
    self.R = self.V_max / self.I_stall
    
    #voltage constant
    self.k_v = (self.V_max - self.I_free * self.R) / self.omega_free
    
    #torque constant
    self.k_t = self.I_stall / self.stall_torque
    
  def torque(self, omega, u):
    u = min(max(u, -self.V_max), self.V_max)
    return (self.n * self.G * self.mu * u) / (self.k_t * self.R) - (self.n * self.G * self.G * self.k_v * omega) / (self.kt * self.R)
        
  def omega(self, tau, u):
    u = min(max(u, -self.V_max), self.V_max)
    return (self.mu * u) / (self.k_v * self.G) - (self.k_t * tau * self.R) / (self.n * self.G * self.G * self.k_v)
    