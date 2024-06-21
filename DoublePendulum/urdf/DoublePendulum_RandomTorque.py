#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Jun 20 19:00:55 2024
Title: Random input to double pendulum to get output data
"""

import pybullet as p
import pybullet_data
import time
import random
import numpy as np
from tqdm import tqdm, trange
import matplotlib.pyplot as plt
plt.style.use('dark_background')

#Simulation Parameters
TimeStep = 0.01

physicsClient = p.connect(p.GUI)
p.setTimeStep(TimeStep)
p.setGravity(0, 0, -9.81)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
planeId = p.loadURDF("plane.urdf")

#Loading model
cubeStartPos1 = [0, 0, 0]
cubeStartOrientation1 = p.getQuaternionFromEuler([0, 0, 0])
Id1 = p.loadURDF("DoublePendulum.urdf", cubeStartPos1, cubeStartOrientation1, useFixedBase = False)

camera = p.resetDebugVisualizerCamera(1, 90, -15, [0, 0, 0])

#Appereance
p.changeVisualShape(Id1, linkIndex = -1, rgbaColor = (1, 0, 1, 1)) #Base
p.changeVisualShape(Id1, linkIndex = 0, rgbaColor = (0, 1, 1, 1)) #l1
p.changeVisualShape(Id1, linkIndex = 1, rgbaColor = (0, 1, 1, 1)) #l2

dof = p.computeDofCount(Id1)
print('dof=', dof)

#Joint friction and damping
p.setJointMotorControl2(Id1, 0, controlMode = p.VELOCITY_CONTROL, force = 0)
p.setJointMotorControl2(Id1, 1, controlMode = p.VELOCITY_CONTROL, force = 0)
p.changeDynamics(Id1, -1, linearDamping = 0, angularDamping = 0)
p.changeDynamics(Id1, 0, linearDamping = 0, angularDamping = 0)
p.changeDynamics(Id1, 1, linearDamping = 0, angularDamping = 0)

#Torque sensors
_ = p.enableJointForceTorqueSensor(Id1, 0)
_ = p.enableJointForceTorqueSensor(Id1, 1)

#Simulation
start = 0.0 
end = 1.0 

steps = int((end - start) / TimeStep) #Número de pasos
t = [0] * steps #Inicializando lista de pasos de tiempo

for s in range(steps):
    t[s] = start + s * TimeStep
    
marker = p.createVisualShape(p.GEOM_SPHERE, radius = 0.005, rgbaColor = (0.905, 0.443, 0.956, 1))

#States
pos1 = []
vel1 = []
avel1 = []
tor1 = []

pos1_theta = []
vel1_theta = []
pos2_theta = []
vel2_theta = []

for r in tqdm(range(len(t))):
    #Reading states
    l1_states = p.getLinkState(Id1, 1, computeLinkVelocity = 1)
    l1_pos = l1_states[0]
    l1_ori = l1_states[1]
    l1_vel = l1_states[6]
    l1_avel = l1_states[7]
    
    l1_euler = p.getEulerFromQuaternion(l1_ori)
    l1_oriy = l1_euler[1] #rad
    l1_oriy_p = l1_states[7][1] #rad/s
    
    print('l1_pos', l1_pos)
    print(l1_oriy)
    print(l1_oriy_p)
    
    #Random Torque
    uop1 = random.uniform(-0.1, 0.1)
    p.setJointMotorControl2(Id1, 0, controlMode = p.TORQUE_CONTROL, force = uop1)
    
    pos1.append(l1_pos[0])
    vel1.append(l1_vel[0])
    avel1.append(l1_avel)
    tor1.append(uop1)
    pos1_theta.append(l1_oriy)
    vel1_theta.append(l1_oriy_p) 

    p.stepSimulation()
    time.sleep(0.1)


np.savetxt('Modulo_data_1000.csv', [p for p in zip(t, pos1_theta, vel1_theta, tor1)], delimiter = ',', fmt = '%s') 

#Plotting
font = {'family': 'sans-serif', 'color': 'aqua',
            'weight': 'bold', 'size': 12}       

fig, (ax1, ax2) = plt.subplots(1, 2)
ax1.set_title("Angular Position \n VS \n Time", fontdict = font)
ax1.set_xlabel('Time', fontdict = font)
ax1.set_ylabel('Angular Position', fontdict = font)
ax2.set_title("Random Torque \n VS \n Time", fontdict = font)
ax2.set_xlabel('Time', fontdict = font)
ax2.set_ylabel('Torque', fontdict = font)
ax1.plot(t, pos1_theta, 'Dw', label = 'Angle') 
ax2.plot(t, tor1, 'Dw', label = 'Random Torque')  
 













