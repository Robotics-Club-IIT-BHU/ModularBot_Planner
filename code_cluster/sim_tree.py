import sys
sys.path.append('C:\\Users\\RTG\\my-pc\\ml\\Modular-Bot\\ModularBot_Planner')
import numpy as np
import math

from mst import Graph

import pybullet as p
import time
import pybullet_data

def create_tree(cubePos):
	#converting list->np array for better handling
	cubePos = np.array(cubePos)

	print(f'Cube shape {cubePos.shape} \n pos\n{cubePos}')

	#-------------building graph and mst--------------
	nodes = cubePos.shape[0]
	g = Graph(nodes)

	#creating graph (edges to nodes)
	#finding distances from one to all other nodes
	dis_oth = np.zeros(nodes,np.float32)

	for i in range(nodes):
		for j in range(i+1,nodes):
			distance = 0
			for k in range(3):
				distance += (cubePos[i][k]-cubePos[j][k])**2

			g.makeEdge(i,j,math.sqrt(distance*1.0))

		for l in range(nodes):
			dis_oth[i]+=g.graph[i][l]

	#print the graph
	print('graph\n',g.graph)

	# finding the cluster head (closest to others)
	head = np.argmin(dis_oth,0)
	print('cluster head index ', head)

	#create minimum spanning tree
	parent = g.mnst(head)
	#print minimum spanning tree
	g.printTree(parent)

	return parent

def sim():
	#setup
	physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
	p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
	p.setGravity(0,0,-10)
	planeId = p.loadURDF("plane.urdf")

	#loading all objects with same orientation
	cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
	#loading positions
	cubeStartPos = np.array([[0,1,1],[1,0,1],[2,0,1],[3,1,1],[3,2,1],[4,0,1],[5,0,1],[6,0,1]])

	#save all boxes
	boxId = []
	#spawn boxes
	for i in range(cubeStartPos.shape[0]):
		boxId.append(p.loadURDF("dabba.urdf",cubeStartPos[i], cubeStartOrientation))
	#simulate
	for i in range (500):
		p.stepSimulation()

		if i%100==0:
			#get cube position and orientation
			cubePos, cubeOrn = [], []
			for i in range(cubeStartPos.shape[0]):
				pos, orn = p.getBasePositionAndOrientation(boxId[i])
				cubePos.append(pos)

			#calling func to create graph
			parent = create_tree(cubePos)

		time.sleep(1./240.)

	#close connection
if __name__ == "__main__":
	sim();time.sleep(1000000000)
