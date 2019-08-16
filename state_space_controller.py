# -*- coding: utf-8 -*-
"""
Created on Sat Apr 28 09:45:27 2018

@author: Maverick Zhang
"""

import numpy


class state_space_controller():
  def __init__(self, A, B, C, D):
    self.A = A
    self.B = B
    self.C = C
    self.D = D
      
  def initialize_state(self):
    """Sets X, Y, and X_hat to zero defaults."""
    self.X = numpy.zeros((self.A.shape[0], 1))
    self.Y = self.C * self.X
    self.X_hat = numpy.zeros((self.A.shape[0], 1))
    
  def update(self, U, x_n = None):
    if x_n is None:
      self.X = self.A * self.X + self.B * U 
      self.Y = self.C * self.X + self.D * U
    else:
      self.X = x_n
      self.Y = self.C * self.X + self.D * U
    
  def predict_observer(self, U):
    """Runs the predict step of the observer update."""
    self.X_hat = (self.A * self.X_hat + self.B * U)

  def correct_observer(self, U):
    """Runs the correct step of the observer update."""
    self.X_hat += numpy.linalg.inv(self.A) * self.L * (
        self.Y - self.C * self.X_hat - self.D * U)

  def update_observer(self, U):
    """Updates the observer given the provided U."""
    self.X_hat = (self.A * self.X_hat + self.B * U +
                  self.L * (self.Y - self.C * self.X_hat - self.D * U))
  
    
  def export(self, pathtofolder):
    numpy.savetxt(pathtofolder + "/" + self.name + "A.csv", self.A, delimiter = ",") #A_d, B_d, C, D, K, L, Kff
    numpy.savetxt(pathtofolder + "/" + self.name + "B.csv", self.B, delimiter = ",")
    numpy.savetxt(pathtofolder + "/" + self.name + "C.csv", self.C, delimiter = ",")
    numpy.savetxt(pathtofolder + "/" + self.name + "D.csv", self.D, delimiter = ",")
    numpy.savetxt(pathtofolder + "/" + self.name + "K.csv", self.K, delimiter = ",")
    numpy.savetxt(pathtofolder + "/" + self.name + "L.csv", self.L, delimiter = ",")
    numpy.savetxt(pathtofolder + "/" + self.name + "Kff.csv", self.Kff, delimiter = ",")
    
    
    