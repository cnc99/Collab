#!/usr/bin/python3
from __future__ import print_function, division, absolute_import

import itertools
import xml.etree.ElementTree as ET

import numpy as np
import pandas as pd
import stl
from pprint import pprint
import pybullet as p
import pybullet_data
import signal
from skopt import gp_minimize, dump, load, forest_minimize, dummy_minimize
from tqdm import tqdm_notebook, tqdm
import os
import sys
from contextlib import contextmanager
from matplotlib import pyplot as plt
# from universal_divergence import estimate as KLD
from continuous_kl import KL_from_distributions as KLD
import logging
import time

#https://github.com/bulletphysics/bullet3/blob/master/examples/pybullet/examples/createMultiBodyLinks.py

PYBULLET_INSTANCE = p.GUI # or p.DIRECT for non-graphical version
WATCHDOG = False
PLOTTING = True
DURATION = 1000
sphere_mass = 0.005
sphere_radius = 0.05
cube_mass = 0.05
cube_height = 0.1


def call_simulator():
    # print("start sim")
    physicsClient = p.connect(PYBULLET_INSTANCE)

    p.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
    p.setGravity(0, 0, -9.8)
    p.setPhysicsEngineParameter(enableFileCaching=0)  # , fixedTimeStep=0.0001)
    planeId = p.loadURDF("plane.urdf")
    #p.changeDynamics(planeId, -1, restitution=.97, linearDamping=0.0, angularDamping=0.0,
    #                 lateralFriction=0.50)


def reset_world():
    p.resetSimulation()


# load object to the simulation
def load_cube(mass):
    boxIdCol = p.createCollisionShape(p.GEOM_BOX, halfExtents=[cube_height, cube_height, cube_height])
    boxIdVis = p.createVisualShape(p.GEOM_BOX, halfExtents=[cube_height, cube_height, cube_height])
    objID = p.createMultiBody(baseMass=mass,
                              baseCollisionShapeIndex=boxIdCol,
                              baseVisualShapeIndex=boxIdVis)
    p.changeDynamics(objID, -1, lateralFriction=0.1)
    return objID

def load_sphere(mass, radius):
    boxIdCol = p.createCollisionShape(p.GEOM_SPHERE, radius=radius)
    boxIdVis = p.createVisualShape(p.GEOM_SPHERE, radius=radius)
    objID = p.createMultiBody(baseMass=mass,
                              baseCollisionShapeIndex=boxIdCol,
                              baseVisualShapeIndex=boxIdVis)
    p.changeDynamics(objID, -1, lateralFriction=0.1)
    return objID

# remove object from simulation
def delete_object(objID):
    p.removeBody(objID)


# orientation and position of an object in the simulation
def get_obj_xy(objID):
    pos, ori = p.getBasePositionAndOrientation(objID)
    pos = pos[0:-1]
    return pos


def gen_run_experiment():

    # Load the cube and the sphere
    cubeId = load_cube(cube_mass)
    sphereId = load_sphere(sphere_mass, sphere_radius)

    # Set the positions of the cube and the sphere
    pxyzc = [0.0, 0.0, cube_height]
    poric = p.getQuaternionFromEuler([0.0, 0.0, 0.0])
    p.resetBasePositionAndOrientation(cubeId, posObj = pxyzc, ornObj=poric)
    pxyzs = [0.0, -1.0, cube_height-(sphere_radius/2)]
    poris = p.getQuaternionFromEuler([0.0, 0.0, 0.0])
    p.resetBasePositionAndOrientation(sphereId, posObj=pxyzs, ornObj=poris)

    # Move the sphere until it reaches the cube
    base_speed = [0.0, 0.01, 0.0]
    p.resetBaseVelocity(sphereId, base_speed)
    success = False

    while not success:
        # Get the Contact Points Between the sphere and the cube
        contact_pts = p.getContactPoints(cubeId, sphereId)

        if contact_pts: #If the list is not empty, the sphere reached the cube
            success = True


        # Apply a force countering gravity so that the sphere doesn't fall
        pos_sphere = p.getBasePositionAndOrientation(sphereId)[0]
        p.applyExternalForce(objectUniqueId=sphereId, linkIndex=-1,
                                                 forceObj=np.array([0, 0, 9.8*sphere_mass]), posObj=pos_sphere, flags=p.WORLD_FRAME)

        p.stepSimulation()
        p.resetBaseVelocity(sphereId, base_speed)


    # The sphere will push the cube for 'duration' time
    duration = 10000

    # Sampling time
    T = 1/240

    # Proportional gain
    kp = 1
    # Integral gain
    ki = 0.01
    k1 = ki*T/2

    # Initialize variables
    i_previous = 0
    error_previous = 0
    counter = 0

    velocities_list = []
    velocities_list_y = []
    velocities_list_cube = []
    velocities_list_cube_y = []
    reference_force = []
    forces_list = []
    error_list= []
    time = []
    base_speed = [0.0, 0.01, 0.0] #5 cm/s in the y direction
    p.resetBaseVelocity(sphereId, linearVelocity = base_speed)
    force = 0.01

    for i in range(duration):

        # Check if there is still contact between the sphere and the cube
        contact_pts = p.getContactPoints(cubeId, sphereId)
        if not contact_pts:  # If the list is empty, there is no contact
            counter = counter + 1

        # If there are more than 100 time steps without contact points, stop the simulation
        if counter > 100:
            break

        # Create the target forces over time
        if i%(240)==0:
            if force == 0.01:
                force = 0.08
            else:
                force = 0.01

        #force = 1.5
        target_force_vector = [0, force, 0]

        # Update the list with the target forces over time
        reference_force.append(force)

        # Update the list with the time samples
        time.append(i/240)

        # Current velocity of the sphere
        velocity = p.getBaseVelocity(sphereId)[0]
        velocities_list.append(np.linalg.norm(velocity))
        velocities_list_y.append(velocity[1])

        # Current velocity of the cube
        velocity_cube = p.getBaseVelocity(cubeId)[0]
        velocities_list_cube.append(np.linalg.norm(velocity_cube))
        velocities_list_cube_y.append(velocity_cube[1])

        # Apply a force countering gravity so that the sphere does not fall
        pos_sphere = p.getBasePositionAndOrientation(sphereId)[0]
        p.applyExternalForce(objectUniqueId=sphereId, linkIndex=-1,
                             forceObj=np.array([0, 0, 9.8 * sphere_mass]), posObj=pos_sphere, flags=p.WORLD_FRAME)

        # Perform the Simulation
        p.stepSimulation()

        # Get the forces the sphere is applying on the cube
        contact_pts = p.getContactPoints(cubeId, sphereId)
        real_force_vector = np.array([0.0, 0.0, 0.0])
        for point in contact_pts:
            real_force_vector += (point[7] / np.linalg.norm(point[7])) * point[9]

        forces_list.append(np.linalg.norm(real_force_vector))

        # Compute the error between the forces
        error = np.linalg.norm(target_force_vector) - np.linalg.norm(real_force_vector)
        error_list.append(error)

        # Calculate the proportional term
        prop = kp*error

        # Calculate the integral term
        i = i_previous + k1 * (error + error_previous)

        # Update variables
        i_previous = i
        error_previous = error

        # Update velocities (always under 40 cm/s)
        u = i + prop

        if velocity[1] + u < 0.7:
            velocity = [0.0, velocity[1] + u, 0.0]

        p.resetBaseVelocity(sphereId, linearVelocity = velocity)



    # Plot the velocities over time
    plt.scatter(time, velocities_list, s=40, c="red", edgecolors='none', label="Sphere")
    plt.scatter(time, velocities_list_cube, s=40, c="blue", edgecolors='none', label="Cube")
    plt.xlabel('Time')
    plt.ylabel('V [m/s]')
    # plt.xlim(0, 2.0)
    # plt.ylim(0, 100)
    plt.title('Velocity over Time')
    plt.legend(loc='best')
    plt.savefig("plots/controller_velocity.png")
    plt.show()

    # Plot the forces over time
    plt.scatter(time, forces_list, s=40, c="red", edgecolors='none', label="Real")
    plt.scatter(time, reference_force, s=40, c="blue", edgecolors='none', label="Target")
    plt.xlabel('Time')
    plt.ylabel('F [N]')
    # plt.xlim(0, 2.0)
    plt.ylim(0, 0.1)
    plt.title('Force Magnitude over Time')
    plt.legend(loc='best')
    plt.savefig("plots/controller_force.png")
    plt.show()

    # Plot the forces over time
    plt.scatter(time, error_list, s=40, c="red", edgecolors='none')
    plt.xlabel('Time')
    plt.ylabel('E')
    # plt.xlim(0, 2.0)
    #plt.ylim(0, 0.1)
    #plt.title('Force Magnitude over Time')
    plt.legend(loc='best')
    plt.savefig("plots/controller_error.png")
    plt.show()

    plt.scatter(time, velocities_list_y, s=40, c="red", edgecolors='none', label="Sphere")
    plt.scatter(time, velocities_list_cube_y, s=40, c="blue", edgecolors='none', label="Cube")
    plt.xlabel('Time')
    plt.ylabel('V [m/s]')
    # plt.xlim(0, 2.0)
    # plt.ylim(0, 100)
    plt.title('Velocity y over Time')
    plt.legend(loc='best')
    plt.savefig("plots/controller_velocity_y.png")
    plt.show()




    return



if __name__ == "__main__":

    logging.info("RIPPE")

    call_simulator()
    gen_run_experiment()
    print('Done')