# -*- coding: utf-8 -*-
"""
Created on Thu Aug  8 15:09:46 2019

@author: Maverick1
"""
import math
import numpy as np

class stribeck_friction_model:
  def __init__(self, F_c, F_v, F_s, v_s, delta):
   self.F_c = F_c
   self.F_v = F_v
   self.F_s = F_s
   self.v_s = v_s
   self.delta = delta
   
  def f(self, v, F):
    vel = abs(v)
    if(vel <= 0.0254):
      if abs(F) <= self.F_s:
        return -F
      else:
        return -self.F_s
    return -np.sign(v) * (self.F_c + (self.F_s - self.F_c) * math.exp(-((vel / self.v_s) ** self.delta)) + self.F_v * vel)
    