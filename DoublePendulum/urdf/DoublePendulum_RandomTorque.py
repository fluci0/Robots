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
Id1 = p.loadURDF("DoublePendulum.urdf", cubeStartPos1, cubeStartOrientation1, useFixedBase = True)

camera = p.resetDebugVisualizerCamera(1, 0, -45, [0, 0, 0])

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
end = 300.0 

steps = int((end - start) / TimeStep) #Número de pasos
t = [0] * steps #Inicializando lista de pasos de tiempo

for s in range(steps):
    t[s] = start + s * TimeStep
    
marker = p.createVisualShape(p.GEOM_SPHERE, radius = 0.005, rgbaColor = (0.905, 0.443, 0.956, 1))

#Vector: to store States
j1_pos = []
j2_pos = []
l1_angpos = []
l2_angpos = []

tor1 = []


for r in tqdm(range(len(t))):
    #Random Torque
    uop1 = random.uniform(-3, 3)
    p.setJointMotorControl2(Id1, 0, controlMode = p.TORQUE_CONTROL, force = uop1)
    
    #Joint States
    j1_states = p.getJointState(Id1, 0)
    j1_p = j1_states[0]
    j1_v = j1_states[1]
    
    j2_states = p.getJointState(Id1, 1)
    j2_p = j2_states[0]
    j2_v = j2_states[1]
    
    #Link states
    L1_states = p.getLinkState(Id1, 0, computeLinkVelocity = 1)
    l1_ori = L1_states[1]
    l1_eori = p.getEulerFromQuaternion(l1_ori)
    j1_theta = l1_eori[1]
    
    L2_states = p.getLinkState(Id1, 1, computeLinkVelocity = 1)
    l2_ori = L2_states[1]
    l2_eori = p.getEulerFromQuaternion(l2_ori)
    j2_theta = l2_eori[1]
    
    #print('l1_eori', l1_eori) #For this case we are interested in rotation angle in Y axis
    
    j1_pos.append(j1_p)
    j2_pos.append(j2_p)
    l1_angpos.append(j1_theta)
    l2_angpos.append(j2_theta)
    tor1.append(uop1)
    
    #print('j1_p', j1_p)
    #print('j1_v', j1_v) 

    p.stepSimulation()
    time.sleep(0.1)

print('CSV creation')

np.savetxt('Pendulum_data_1000.csv', [p for p in zip(t, j1_pos, j2_pos, l1_angpos, l2_angpos, tor1)], delimiter = ',', fmt = '%s') 


#Plotting
font = {'family': 'sans-serif', 'color': 'aqua',
            'weight': 'bold', 'size': 12}       

fig, (ax1, ax2, ax3) = plt.subplots(1, 3)
ax1.set_title("[L1] Angular Position \n VS \n Time", fontdict = font)
ax1.set_xlabel('Time', fontdict = font)
ax1.set_ylabel('Angular Position', fontdict = font)
ax2.set_title("[L2] Angular Position \n VS \n Time", fontdict = font)
ax2.set_xlabel('Time', fontdict = font)
ax2.set_ylabel('Angular Position', fontdict = font)
ax3.set_title("Random Torque \n VS \n Time", fontdict = font)
ax3.set_xlabel('Time', fontdict = font)
ax3.set_ylabel('Torque', fontdict = font)

ax1.plot(t, l1_angpos, 'Dw', label = 'Angle') 
ax2.plot(t, l2_angpos, 'Dw', label = 'Angle') 
ax3.plot(t, tor1, 'Dw', label = 'Random Torque')  
 













