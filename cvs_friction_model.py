# -*- coding: utf-8 -*-
"""
Created on Tue Aug  6 17:47:38 2019

@author: Maverick1
"""

import numpy as np

class cvs_friction_model:
  def __init__(self, F_c, F_v, F_s):
    self.F_c = F_c
    self.F_v = F_v
    self.F_s = F_s
    
  def f(self, v, F):
    if abs(v) <= 0.254: #1.0E-8:
      if abs(F) <= self.F_c + self.F_s:
        return -F
      else:
        return -self.F_c * np.sign(v)
    elif abs(v) > 0.00254 and abs(v) <= 0.0254:
      return -self.F_c * np.sign(v)
    return -self.F_c * np.sign(v) - self.F_v * v