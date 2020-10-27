import sys
sys.path.append('C:\\Users\\RTG\\my-pc\\ml\\Modular-Bot\\ModularBot_Planner')
import numpy as np
import math

from mst import Graph 
from basic_sim import sim

#starting sim and ending
cubePos, cubeOrn = sim()
#converting list->np array for better handling
cubePos = np.array(cubePos)
cubeOrn = np.array(cubeOrn)

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
