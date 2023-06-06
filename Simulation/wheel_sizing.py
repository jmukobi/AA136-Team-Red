"""
This script is used to size the flywheel for the reaction wheel assembly.
"""

import numpy as np
# L = Iw

#I_sat * w_sat = I_wheel * w_wheel

#I_wheel = I_sat * w_sat /w_wheel

w_wheel = 15000/60 #rad/s
w_sat = (1/360)*2*np.pi # rad/s
I_sat = .05 #kgm2
I_wheel = I_sat*w_sat/w_wheel
print(f"Flywheel inertia must be {I_wheel} kg*m^2")

#Size flywheel in PLA

#Moment of inertia of a cylinder
#I = .5*M*R^2
R = .014
M = (2*I_wheel)/(R**2)
rho = 1240
V = M/rho

#V = pi*r^2*h
h = V/(np.pi*R**2)

print(f"Flywheel radius = {R} m\nFlywheel height = {h} m")
