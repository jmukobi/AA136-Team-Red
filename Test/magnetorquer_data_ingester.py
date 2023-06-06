"""
This program reads in the data from magnetorquer tests in .csv form and plots it.
"""

import math
import csv
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def jade_stuff():
    # Number Coils
    M = 0.06  # moment in A/m^2, double calculated
    I = 0.6   # half of max of 1.2, A

    # Cylindrical magnetorquer
    r = 0.008  # radius, m
    A_C = math.pi * r**2  # area cylinder, m^2
    N_C = M / (I * A_C)  # number coils cylinder
    l_C = N_C * 2 * math.pi * r  # length of wire cylinder, m

    # Square Magnetorquer
    s = 0.072  # side length, m
    A_S = s**2  # area square, m^2
    N_S = M / (I * A_S)  # number coils square
    l_S = N_S * 4 * s  # length of wire square, m

    # Voltage
    R = 0.9088  # resistance per m
    Ra = l_C * R  # resistance cylinder coil (longest)
    V = I * Ra  # voltage to coils

    print(Ra)
    print("Number of coils (cylinder):", N_C)
    print("Length of wire (cylinder):", l_C)
    print("Number of coils (square):", N_S)
    print("Length of wire (square):", l_S)
    print("Voltage to coils:", V)

def plot_data():
    # Read data from CSV file
    filename = 'volt_meas.csv'
    with open(filename, 'r') as file:
        reader = csv.reader(file)
        data = list(reader)

    # Extract relevant columns from the data
    trial_nos = [100*float(row[1]) for row in data[1:]]
    moments_1 = [float(row[3]) for row in data[1:]]
    moments_2 = [float(row[5]) for row in data[1:]]
    moments_3 = [float(row[7]) for row in data[1:]]
    average_moments = [float(row[8]) for row in data[1:]]

    # Plot moments and average moments
    plt.plot(trial_nos, moments_1, label='Moment 1')
    plt.plot(trial_nos, moments_2, label='Moment 2')
    plt.plot(trial_nos, moments_3, label='Moment 3')
    plt.plot(trial_nos, average_moments, label='Average Moments')

    # Set plot labels and title
    plt.xlabel('Throttle Setting (%)')
    plt.ylabel('Moment (A*m^2))')
    plt.grid()
    plt.title('Calculated Moment from Measure Voltage')

    # Add legend
    plt.legend()

    # Display the plot
    plt.show()


    #Measured B field Vectors

    # Read data from CSV file
    filename = 'mag_meas.csv'
    with open(filename, 'r') as file:
        reader = csv.reader(file)
        data = list(reader)

    # Extract vector components from the data
    labels = [row[0] for row in data[1:]]
    x_values = [float(row[1]) for row in data[1:]]
    y_values = [float(row[2]) for row in data[1:]]
    z_values = [float(row[3]) for row in data[1:]]

    # Create a 3D plot
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Plot vectors
    ax.quiver(0, 0, 0, x_values, y_values, z_values, label='Vectors')

    # Set plot labels and title
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('Vector Visualization')

    # Add labels for each vector
    for label, x, y, z in zip(labels, x_values, y_values, z_values):
        ax.text(x, y, z, label)

    max_length = 200
    #ax.set_xlim(-max_length, max_length)
    #ax.set_ylim(-max_length, max_length)
    #ax.set_zlim(-max_length, max_length)

    # Add legend
    ax.legend()

    # Display the plot
    plt.show()



    #Plot Net B Field

    # Read data from CSV file
    filename = 'mag_net_meas.csv'
    with open(filename, 'r') as file:
        reader = csv.reader(file)
        data = list(reader)

    # Extract vector components from the data
    labels = [row[0] for row in data[1:]]
    x_values = [float(row[1]) for row in data[1:]]
    y_values = [float(row[2]) for row in data[1:]]
    z_values = [float(row[3]) for row in data[1:]]

    # Create a 3D plot
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Plot vectors
    ax.quiver(0, 0, 0, x_values, y_values, z_values, label='Vectors')

    # Set plot labels and title
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('Vector Visualization')

    # Add labels for each vector
    for label, x, y, z in zip(labels, x_values, y_values, z_values):
        ax.text(x, y, z, label)

    max_length = 200
    #ax.set_xlim(-max_length, max_length)
    #ax.set_ylim(-max_length, max_length)
    #ax.set_zlim(-max_length, max_length)

    # Add legend
    ax.legend()

    # Display the plot
    plt.show()

    return


def main():
    #jade_stuff()
    plot_data()

main()