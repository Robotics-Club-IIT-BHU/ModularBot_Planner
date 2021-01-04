import numpy as np
import math

import pybullet as p
import time
import pybullet_data

from sklearn.cluster import KMeans

from mst import Graph


def create_tree(cubePos, dicts):
	#converting list->np array for better handling
	cubePos = np.array(cubePos)

	print(f'--> Cube shape {cubePos.shape} \n pos\n{cubePos}')

	#-------------building graph and mst--------------
	nodes = cubePos.shape[0]
	g = Graph(nodes, dicts)

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
	print('-------------graph -------------\n',g.graph)

	# finding the cluster head (closest to others)
	head = np.argmin(dis_oth,0)
	print(f'********* cluster head - index:{head} boxId:{dicts[head]} *********')

	#create minimum spanning tree
	parent = g.mnst(head)
	#print minimum spanning tree
	g.printTree(parent)

	return parent

def form_cluster(X, n_clusters=2):
	Kmean = KMeans(n_clusters=n_clusters)
	Kmean.fit(X)
	print(f"--> cluster centers {Kmean.cluster_centers_}")
	print(f"--> cluster label {Kmean.labels_}")
	return Kmean.labels_

def sim(num_bots=8, seed=0, num_clusters=2, debug=False):
	#random seed
	np.random.seed(seed)
	#setup
	physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
	p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
	p.setGravity(0,0,-10)
	planeId = p.loadURDF("plane.urdf")

	#loading all objects with same orientation
	cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
	#loading positions
	area = num_bots/10
	cubeStartPos = np.random.randint(low=[-area,-area,0], high=[area,area,1], size=(num_bots,3),dtype=np.int8)
	print("cube poses ",cubeStartPos)

	#save all boxes
	boxId = []
	#spawn boxes
	for i in range(num_bots):
		boxId.append(p.loadURDF("dabba.urdf",cubeStartPos[i], cubeStartOrientation))
	#simulate
	for i in range (100):
		p.stepSimulation()

		if i%100==0:
			#get cube position and orientation
			cubePos, cubeOrn = [], []
			for i in range(num_bots):
				pos, orn = p.getBasePositionAndOrientation(boxId[i])
				cubePos.append(pos)

			on_whole = {}
			for i in range(num_bots):
				on_whole[i]=boxId[i]

			km_labels = form_cluster(cubePos,num_clusters)
			groups = [[] for _ in range(num_clusters)]
			dictionaries = [[] for _ in range(num_clusters)]
			parents = [[] for _ in range(num_clusters)]

			for i in range(num_clusters):
				dd = {}
				k=0
				for j in range(len(km_labels)):
					if(km_labels[j]==i):
						#splitting into their respective clusters
						groups[i].append(cubePos[j])
						#storing their boxid
						dd[k]=boxId[j]
						k+=1
				#box ids
				dictionaries[i].append(dd)

				# calling func to create graph
				for _ in range(3):
					print("")
				print(f"------------- %%creating cluster #{i+1} %% -------------")
				#store parent connections
				parent = create_tree(groups[i],dictionaries[i][0])
				parents[i].append(parent)
				#add debugging lines
				if debug==True:
					r = int(np.random.normal(0,0.5,1)*255)
					g = int(np.random.normal(0,0.8,1)*255)
					b = int(np.random.normal(0,0.7,1)*255)
					for o in range(len(parent)):
						p.addUserDebugLine(groups[i][o],groups[i][parent[o]],[r,g,b])

			#pause
			print("**Enter q to continue:")
			while input() == 'q':
				break

		time.sleep(1./240.)

	#close connection
if __name__ == "__main__":
	sim();time.sleep(1000000000)
