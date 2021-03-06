import pybullet as p
import time
import numpy as np

import os
import sys
sys.path.append('/media/aditya/Work/THESIS/MT_Thesis/Quadruped/Pybullet/pybullet_robots/data/quadroit')
from Instinct import *

p.connect(p.GUI)
plane = p.loadURDF("plane.urdf")
p.setGravity(0,0,-9.8)
p.setTimeStep(1./500)
#p.setDefaultContactERP(0)
#urdfFlags = p.URDF_USE_SELF_COLLISION+p.URDF_USE_SELF_COLLISION_EXCLUDE_ALL_PARENTS 
urdfFlags = p.URDF_USE_SELF_COLLISION
quadruped = p.loadURDF("quadroit/quadv1.urdf",[0,0,.5],[0,0,0.5,0], flags = urdfFlags,useFixedBase=False)

#enable collision between lower legs

for j in range (p.getNumJoints(quadruped)):
    print(p.getJointInfo(quadruped,j))
    # print(p.getDynamicsInfo(quadruped,j))

#2,5,8 and 11 are the lower legs
lower_legs = [2,5,8,11]
for l0 in lower_legs:
	for l1 in lower_legs:
		if (l1>l0):
			enableCollision = 1
			print("collision for pair",l0,l1, p.getJointInfo(quadruped,l0)[12],p.getJointInfo(quadruped,l1)[12], "enabled=",enableCollision)
			p.setCollisionFilterPair(quadruped, quadruped, l0,l1,enableCollision)

jointIds=[]
paramIds=[]
jointOffsets=[]
jointDirections= [1,-1,1,1,-1,1,-1,1,-1,-1,1,-1]
jointAngles=[0,0,0,0,0,0,0,0,0,0,0,0]
# jointScale = np.pi/180
jointScale = 1
W_DOF = 12


# fix joints [0,3,6,9] : no hip movement
# try changeConstraint
# if W_DOF == 8:
#         joints[0:3:9] = 0

dir_flag = 1
for i in range (4):
        jointOffsets.append(0)
        jointOffsets.append(0)
        if i>=2:
            dir_flag=-1
        jointOffsets.append(dir_flag*0)

maxForceId = p.addUserDebugParameter("maxForce",0,500,400)
restitutionId = p.addUserDebugParameter("restitution",0,1,0.1)
restitutionThresholdId = p.addUserDebugParameter("res. vel. thres",0,100,100)
p.getCameraImage(480,320)
p.setRealTimeSimulation(1)
# while(1):
for j in range (p.getNumJoints(quadruped)):
        restitution = p.readUserDebugParameter(restitutionId)
        restitutionThreshold = p.readUserDebugParameter(restitutionThresholdId)
        p.setPhysicsEngineParameter(restitutionVelocityThreshold = restitutionThreshold)
        p.changeDynamics(quadruped,j,linearDamping=0, angularDamping=0)
        p.changeDynamics(plane, -1, restitution=restitution)
        p.changeDynamics(quadruped, j, restitution=restitution)
        p.changeDynamics(plane, -1, lateralFriction=1)
        p.changeDynamics(quadruped,j,lateralFriction=1)
        info = p.getJointInfo(quadruped,j)
        #print(info)
        jointName = info[1]
        jointType = info[2]
        if (jointType==p.JOINT_PRISMATIC or jointType==p.JOINT_REVOLUTE):
                jointIds.append(j)
        if (jointType==p.JOINT_FIXED):
                p.changeDynamics(quadruped,j,lateralFriction=1)
                # restitutionThreshold = p.readUserDebugParameter(restitutionThresholdId)
                # p.changeDynamics(plane, -1, restitution=restitution)
                p.changeDynamics(quadruped,j,rollingFriction=0.2)
                p.changeDynamics(quadruped,j,spinningFriction=0.2)  
                p.changeDynamics(quadruped,j,contactDamping=1000,contactStiffness=0)    

		
# p.getCameraImage(480,320)
# p.setRealTimeSimulation(1)

joints=[]

# calibration position
# pose input
# targetPos = float(joints[j])
with open("testJ22_L.txt","r") as filestream:
	for line in filestream:
		maxForce = p.readUserDebugParameter(maxForceId)
		currentline = line.split(",")
		# frame = currentline[0]
		# t = currentline[1]
		joints=currentline[0:12]
		for j in range (12):
			targetPos = float(joints[j])
			p.setJointMotorControl2(quadruped,jointIds[j],p.POSITION_CONTROL,jointDirections[j]*targetPos+jointOffsets[j], force=maxForce)
		p.stepSimulation()
		for lower_leg in lower_legs:
			#print("points for ", quadruped, " link: ", lower_leg)
			pts = p.getContactPoints(quadruped,-1, lower_leg)
			#print("num points=",len(pts))
			#for pt in pts:
			#	print(pt[9])
		time.sleep(1./1000.)

# motion1 = instincts['wkL']
# lineiter = iter(motion1.splitlines())
# next(lineiter)
# dof_ord = [0,0,4,0,1,5,0,3,7,0,2,6]

# for i in range(100):
#     print(i)
#     maxForce = p.readUserDebugParameter(maxForceId)
#     lineiter = iter(motion1.splitlines())
#     next(lineiter)
#     for line in lineiter:
        
#         currentline = line.split(",")
#         #   print(currentline[:])
#         # frame = currentline[0]
#         # t = 50*currentline[1]
#         joints = currentline[0:8]
#         for j in range (12):
#             if (j==0 or j==3 or j==6 or j==9):
#                     targetPos = 0
#             else:
#                     targetPos = float(joints[dof_ord[j]])
#                     print(j,targetPos)
#             p.setJointMotorControl2(quadruped,jointIds[j],p.POSITION_CONTROL,jointDirections[j]*targetPos*jointScale+jointOffsets[j], force=maxForce)
#             # print(jointDirections[j]*targetPos*jointScale+jointOffsets[j])
#         p.stepSimulation()
#         # for lower_leg in lower_legs:
#             #print("points for ", quadruped, " link: ", lower_leg)
#             # pts = p.getContactPoints(quadruped,-1, lower_leg)
#             #print("num points=",len(pts))
#             #for pt in pts:
#             #	print(pt[9])
#         time.sleep(1./200.)
    
    # joints=[]
# index = 0
# for j in range (p.getNumJoints(quadruped)):
#         # p.changeDynamics(quadruped,j,linearDamping=0, angularDamping=0)
#         info = p.getJointInfo(quadruped,j)
#         js = p.getJointState(quadruped,j)
#         jointName = info[1]
#         jointType = info[2]
#         if (jointType==p.JOINT_PRISMATIC or jointType==p.JOINT_REVOLUTE):
#                 print(j,(js[0]-jointOffsets[index])/jointScale)
#                 paramIds.append(p.addUserDebugParameter(jointName.decode("utf-8"),-90,90,(js[0]/jointScale)))
#                 # -jointOffsets[index]
#                 index=index+1

# index = 0
# for j in range (p.getNumJoints(quadruped)):
#         p.changeDynamics(quadruped,j,linearDamping=0, angularDamping=0)
#         info = p.getJointInfo(quadruped,j)
#         js = p.getJointState(quadruped,j)
#         # print(info)
#         jointName = info[1]
#         jointType = info[2]
#         if (jointType==p.JOINT_PRISMATIC or jointType==p.JOINT_REVOLUTE):
#                 paramIds.append(p.addUserDebugParameter(jointName.decode("utf-8"),-90,90,(js[0])/(jointDirections[index])))
#                 # -jointOffsets[index]
#                 index=index+1


p.setRealTimeSimulation(1)

while (1):
	
	for i in range(len(paramIds)):
		c = paramIds[i]
		targetPos = p.readUserDebugParameter(c)
		maxForce = p.readUserDebugParameter(maxForceId)
		p.setJointMotorControl2(quadruped,jointIds[i],p.POSITION_CONTROL,jointDirections[i]*targetPos*jointScale+jointOffsets[i], force=maxForce)
	
