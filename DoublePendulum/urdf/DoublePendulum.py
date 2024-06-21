# -*- coding: utf-8 -*-
"""
Created on Thu Dec 20 19:12:11 2024

@author: fluci0
"""
import pybullet as p
import pybullet_data
import math as m

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
print("data path: %s " % pybullet_data.getDataPath())
planeId = p.loadURDF("plane.urdf")

#Parameters of the Double pendulum
g = 9.81
m1 = 0.35 #Mass of arm 1 Obtained from SW
m2 = 0.35 #Mass of arm 2 Obtained from SW
l1 = 0.2 #Length of the arm 1
l2 = 0.2 #Lenght of the arm 2
lc1 = l1/2 #Distance to the center of the arm 1
lc2 = l2/2 #Distance to the center of the arm 2
I1 = (m1*l1**2)/3 #Moment of inertia of the arm 1
I2 = (m2*l2**2)/3 #Moment of inertia of the arm 2

#Loading the model of the pendulum
cubeStartPos1 = [0, 0, 0]
cubeStartOrientation1 = p.getQuaternionFromEuler([0, 0, 0])
boxId = p.loadURDF("DoublePendulum.urdf",cubeStartPos1, cubeStartOrientation1)
basePosOri = p.getBasePositionAndOrientation(boxId)
print("Base position and orientation :", basePosOri)

#Simulation Parameters
p.setRealTimeSimulation(1) #1: enable , 2: disable, default time step is 1/240 seconds
p.setGravity(0, 0, -9.81)

#Changing colors of the links
p.changeVisualShape(boxId,linkIndex = -1, rgbaColor = (1, 0, 1, 1)) #Base
p.changeVisualShape(boxId,linkIndex = 0, rgbaColor = (0, 1, 1, 1)) #A1
p.changeVisualShape(boxId,linkIndex = 1, rgbaColor = (1, 1, 1, 1)) #A2

#Creating a constraint between the plane and the base of the pendulum
c1 = p.createConstraint(boxId, -1, -1, -1, p.JOINT_FIXED, [0, 0, 0], [0, 0, 0], [0, 0, 0])

#Knowing the initial states of the Simulation
arm1_state = p.getJointState(boxId, linkIndex = 0)
arm2_state = p.getJointState(boxId, linkIndex = 1)
arm1_WOrientation = arm1_state[5]
arm2_WOrientation = arm2_state[5]
arm1_WOrientationEU = p.getEulerFromQuaternion(arm1_WOrientation)
arm2_WOrientationEU = p.getEulerFromQuaternion(arm2_WOrientation)
print("arm1_state: ", arm1_state)
print("arm2_state: ", arm2_state)
print("arm1_WOrientation: ", arm1_WOrientation)
print("arm2_WOrientation: ", arm2_WOrientation)
print("arm1_WOrientationEU: ", arm1_WOrientationEU)
print("arm2_WOrientationEU: ", arm2_WOrientationEU)

#Unlocking motors
n = 2
forces_zeros = [0]*n
#p.setJointMotorControlArray(boxId, jointIndices=[0, 1], controlMode=p.VELOCITY_CONTROL, forces=forces_zeros)

#Initial conditions of the double pendulum
Dox01=45*m.pi/180 #Initial condition -- Angular position of arm 1
Dox02=0*m.pi/180 #Initial condition -- Angular velocity of arm 1
Dox03=-45*m.pi/180 #Initial condition -- Angular position of arm 2
Dox04=0*m.pi/180 #Initial condition -- Angular velocity of arm 2

#Operation Points
Doxop1=0*m.pi/180 #Angular position of operating (arm 1)
Doxop2=0*m.pi/180 #Angular velocity of operating (arm 1)
Doxop3=0*m.pi/180 #Angular position of operating (arm 1)
Doxop4=0*m.pi/180 #Angular velocity of operating (arm 1)

#Setting our model in the conditions we need
#p.setJointMotorControl2(boxId, 0, controlMode = p.POSITION_CONTROL, targetPosition = Doxop1)
#p.setJointMotorControl2(boxId, 1, controlMode = p.POSITION_CONTROL, targetPosition = Doxop3)

#Static control
a=(m1*((lc1)**2)+m2*((l1)**2)+m2*((lc2)**2)+2*m2*l1*lc2*m.cos(Doxop3)+I1+I2)
b=(m2*((lc2)**2)+m2*l1*lc2*m.cos(Doxop3)+I2)
c=(-g*lc2*m2*m.sin(Doxop1+Doxop3))-(g*l1*m2*m.sin(Doxop1))-(g*lc1*m1*m.sin(Doxop1))-(Doxop4*l1*lc2*m2*m.sin(Doxop3)*(2*Doxop2+Doxop4))
d=(m2*(lc2)**2+I2)
e=(m2*l1*lc2*(Doxop2)**2*m.sin(Doxop3)-m2*g*lc2*m.sin(Doxop1+Doxop3))
Uop=((b*e)-(d*c))/-d
print("Static Control torque = ", Uop)

#p.setJointMotorControl2(boxId, 0, controlMode = p.POSITION_CONTROL, targetPosition = Doxop1, force = Uop)

