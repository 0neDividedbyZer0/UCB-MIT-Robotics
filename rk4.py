# -*- coding: utf-8 -*-
"""
Created on Tue Aug  6 18:14:12 2019

@author: Maverick1
"""

def rk4(f, t_n, x_n, u_n, h):
  k1 = h * f(t_n, x_n, u_n)
  k2 = h * f(t_n + 0.5 * h , x_n + 0.5 * k1, u_n)
  k3 = h * f(t_n + 0.5 * h , x_n + 0.5 * k2, u_n)
  k4 = h * f(t_n + h, x_n + k3, u_n)
  return x_n + ((k1 + 2.0 * k2 + 2.0 * k3 + k4) / 6.0)

def euler(f, t_n, x_n, u_n, h):
  return x_n + h * f(t_n, x_n, u_n)