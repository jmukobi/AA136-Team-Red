"""
CubeSat Attitude Simulator

This program simulates the attitude of a CubeSat using Euler's equations of motion.
It is used to test control algorithms for an ADCS system.

Author: Jacob Mukobi
"""

import numpy as np
import matplotlib.pyplot as plt
import random

def attitude_propagator(dt, t_end, w0, I, torque_func, scenario, tolerance=1e-4):
    """
    Propagates the attitude of a CubeSat over time using Euler's equations of motion.

    Args:
        dt: Time step size in seconds.
        t_end: End time of the simulation in seconds.
        w0: Initial angular velocity vector in rad/s.
        I: Inertia matrix of the CubeSat in kg.m^2.
        torque_func: Function that takes the current time t in seconds as input and returns the torque vector in N.m.

    Returns:
        R: Numpy array of shape (3, 3, num_steps) representing the rotation matrix at each time step.
    """
    num_steps = int(t_end / dt)
    R = np.zeros((3, 3, num_steps+1))
    w = w0.copy()
    dw=0
    Is_Settled = False
    dw_dt = 0
    w_list = []

    # Initialize the rotation matrix as the identity matrix
    R[:, :, 0] = np.eye(3)
    # Initialize the magnitude of angular momentum as the product of the inertia matrix and the initial angular velocity
    H = [np.linalg.norm(I @ w)]

    # Perform numerical integration using the fourth-order Runge-Kutta method
    for i in range(num_steps):
        # Compute the current time
        t = i * dt

        # Print progress
        if t%30 == 0:
            print(f"{int(100*t/t_end)}% done")

        # Find change in angle for PID control
        w_list.append(w)
        if len(w_list) > 100:
            w_list.pop(0)
        theta = sum(w_list)*dt
        theta += w*dt

        # Comput torque vector
        tau = torque_func(t, theta, w, dw_dt, scenario, law="proportional")
        #print(np.linalg.norm(tau))
        # Compute the time derivative of the angular velocity
        dw_dt = np.linalg.inv(I) @ (tau - np.cross(w, I @ w))

        # Use the fourth-order Runge-Kutta method to obtain the change in angular velocity
        k1 = dw_dt
        k2 = np.linalg.inv(I) @ (tau - np.cross(w + 0.5*dt*k1, I @ (w + 0.5*dt*k1)))
        k3 = np.linalg.inv(I) @ (tau - np.cross(w + 0.5*dt*k2, I @ (w + 0.5*dt*k2)))
        k4 = np.linalg.inv(I) @ (tau - np.cross(w + dt*k3, I @ (w + dt*k3)))
        dw = (1/6) * dt * (k1 + 2*k2 + 2*k3 + k4)
        
        # Update the angular velocity
        w = w + dw

        # Use the rotation matrix update equation to obtain the updated rotation matrix
        exp_w_dt = np.eye(3) + np.sin(dt*np.linalg.norm(w))/np.linalg.norm(w) * np.array([[0, -w[2], w[1]], \
            [w[2], 0, -w[0]], [-w[1], w[0], 0]]) + ((1-np.cos(dt*np.linalg.norm(w)))/(np.linalg.norm(w)**2)) * \
            np.array([[w[0]**2, w[0]*w[1], w[0]*w[2]], [w[0]*w[1], w[1]**2, w[1]*w[2]], [w[0]*w[2], w[1]*w[2], w[2]**2]])
        R[:, :, i+1] = R[:, :, i] @ exp_w_dt
        
        # Append angular momentum to list
        H.append(np.linalg.norm(I @ w))

        # Detect performance flag
        if np.linalg.norm(I @ w) < tolerance and Is_Settled == False:
            print(f"System settled within tolerance at time {t} seconds.")
            Is_Settled = True

    return R, H

def rotmat_to_euler(R):
    """
    Converts a rotation matrix to 3D Euler angles.

    Args:
        R: Numpy array of shape (3, 3, num_steps) representing the rotation matrix at each time step.

    Returns:
        euler_angles: Numpy array of shape (3, num_steps) representing the Euler angles (roll, pitch, yaw) at each time step.
    """
    num_steps = R.shape[2]
    euler_angles = np.zeros((3, num_steps))

    for i in range(num_steps):
        R_i = R[:, :, i]
        roll = np.rad2deg(np.arctan2(R_i[2, 1], R_i[2, 2]))
        pitch = np.rad2deg(-np.arcsin(R_i[2, 0]))
        yaw = np.rad2deg(np.arctan2(R_i[1, 0], R_i[0, 0]))
        euler_angles[:, i] = np.array([roll, pitch, yaw])

    return euler_angles

def torque_func(time, theta, w, alpha, scenario, law):
    
    t = [0, 0, 0]

    max_torque = 1.65e-5 #max torque in Nm
    
    if scenario == "Detumble":
        #print("detumbling")
        if law == "proporational":
            #simple inverse proportional control
            t = -w
        if law == "PID":
            #PID control
            t = -1*w - 0.1*theta + 0.01*alpha 

    elif scenario == "Point":
        #print("pointing")
        target = np.array([1.5, 1, 2])
        error = target - theta
        #print(f"error : {error}")
        #simple inverse proportional control
        #print("proportional")
        Kp = .1*max_torque
        Kd = .01*max_torque
        #Ki = .001
        t = Kp*error + Kd*alpha

    for i in range(3):
        if t[i] > max_torque:
            t[i] = max_torque
            #print("Actuator saturation")
        elif t[i] < -max_torque:
            t[i] = -max_torque
            #print("Actuator saturation")

    #print(t)

    #t = inject_disturbances(time, t)
    return t
    
def inject_disturbances(time, t):
    for i in range(3):
        t[i] += random.randrange(-1, 1)*1e-6
    return t

def main():
    # Define the simulation parameters

    #scenario = "Detumble"
    scenario = "Point"

    if scenario == "Detumble":
            w0 = np.array([.09, .09, .09])  # initial angular velocity in rad/s
    elif scenario == "Point":
            w0 = np.array([0, 0, 0])

    dt = 0.05  # seconds
    t_end = 60*30  # seconds
    I = np.diag([.042, .056, .013])  # inertia matrix in kg.m^2

    # Propagate the attitude and convert the rotation matrix to Euler angles
    R, H = attitude_propagator(dt, t_end, w0, I, torque_func, scenario)

    euler_angles = rotmat_to_euler(R) 
    
    # Plot the Euler angles over time
    plt.subplot(2, 1, 1)
    t = np.linspace(0, t_end, euler_angles.shape[1])
    plt.plot(t, euler_angles[0, :],"-", label='Roll')
    plt.plot(t, euler_angles[1, :],"-", label='Pitch')
    plt.plot(t, euler_angles[2, :],"-", label='Yaw')
    plt.xlabel('Time (s)')
    plt.ylabel('Angle (degrees)')
    plt.grid()
    plt.title("Euler Angles of Beluga Over Time")
    plt.legend()
    
    # Plot the angular momentum over time
    plt.subplot(2, 1, 2)
    plt.plot(t, H)
    plt.xlabel('Time (s)')
    plt.ylabel('Angular momentum (kg*m^2/s)')
    plt.grid()
    plt.title("Angular Momentum of Beluga Over Time")
    plt.show()

if __name__ == '__main__':
    main()