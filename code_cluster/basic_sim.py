import pybullet as p
import time
import pybullet_data
import numpy as np

#setup
physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-10)
planeId = p.loadURDF("plane.urdf")

#loading all objects with same orientation
cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
#loading positions
cubeStartPos = np.array([[0,0,1],[1,0,2],[0,2,1],[0,0,1],[0,1,1],[1,0,1],[0,0,1]])

print(cubeStartPos.shape)
#save all boxes 
boxId = []
#spawn boxes
for i in range(cubeStartPos.shape[0]):
	boxId.append(p.loadURDF("dabba.urdf",cubeStartPos[i], cubeStartOrientation))
#simulate
for i in range (10000):
    p.stepSimulation()
    time.sleep(1./240.)

#get cube position and orientation
cubePos, cubeOrn = [], []
for i in range(cubeStartPos.shape[0]):
	pos, orn = p.getBasePositionAndOrientation(boxId[0])
	cubePos.append(pos)
	cubeOrn.append(orn)

print(cubePos,cubeOrn)

#close connection
p.disconnect()