import numpy as np
import math

import pybullet as p
import time
import pybullet_data

from sklearn.cluster import KMeans

try:
    from .mst import Graph
except:
    from mst import Graph

def create_tree(cubePos, dicts, debug=False):
    """
    Create minimum spaning tree with nodes

    Parameters:
    cubePos : position of all nodes in 3d space
    dicts : mapping from cluster index to box id to uniquely identify boxes

    Return :
    parent : list of parent vertex indicies
    """

    #converting list->np array for better handling
    cubePos = np.array(cubePos)
    if debug:
        print(f'--> Cube shape {cubePos.shape} \n pos\n{cubePos}')
	#-------------building graph and minimum spaning tree--------------
    nodes = cubePos.shape[0]
    g = Graph(nodes, dicts)

	#creating graph (edges to nodes)
	#finding distances between each node
    dis_oth = np.zeros(nodes,np.float32)

    for i in range(nodes):
        for j in range(i+1,nodes):
            distance = 0
            for k in range(2):
                distance += (cubePos[i][k]-cubePos[j][k])**2
            #creating edge with weight distance
            g.makeEdge(i,j,math.sqrt(distance*1.0))

        for l in range(nodes):
            dis_oth[i]+=g.graph[i][l]

	#print the graph
    if debug: print('-------------graph -------------\n',g.graph)

	# finding the cluster head (closest to others)
    head = np.argmin(dis_oth,0)
    if debug: print(f'********* cluster head - index:{head} boxId:{dicts[head]} *********')

	#create minimum spanning tree
    parent = g.mnst(head)
	#print minimum spanning tree
    if debug:
        g.printTree(parent)

    return parent

def form_cluster(X, n_clusters=2, debug=False):
    """
    Form clusters from given points in 3d space using k means algorithm

    Parameters:
    X : collection of 3d points to be put into clusters
    n_clusters : equal to argument num_clusters, number clusters to be formed

    Return:
    labels : list of labels of cluster to which the points belong
    """
    Kmean = KMeans(n_clusters=n_clusters)
    Kmean.fit(X)
    if debug:
        print(f"--> cluster centers {Kmean.cluster_centers_}")
        print(f"--> cluster label {Kmean.labels_}")
    return Kmean.labels_

def cluster(cubePos, botIds, num_clusters, debug=False, pClient=None):
    """
    Clusters the objects and creates the Tree for each cluster

    Parameters:
    cubePos : set of 3D coordinates of each bot
    botIds : spawned Id each of these bots
    num_clusters : Numbers of clusters the bots must be divided into
    debug : Flag to enable debugging or auxilary outputs

    Return:
    parents : For each cluster contains details of its Minimum Spanning Tree by having parent index of each bot
    groups : Stores the 3D coordinates for each bot in each cluster
    dictionaries : Maps from the cluster index to the spawned id for each bot in each cluster
    """
    on_whole = {}
    for i in range(len(botIds)):
        on_whole[i]=botIds[i]

    km_labels = form_cluster(cubePos,num_clusters)
    groups = [[] for _ in range(num_clusters)]
    dictionaries = [[] for _ in range(num_clusters)]
    parents = [[] for _ in range(num_clusters)]

    if debug:p.removeAllUserDebugItems()
    for i in range(num_clusters):
        dd = {}
        k=0
        for j in range(len(km_labels)):
            if(km_labels[j]==i):
                #splitting into their respective clusters
                groups[i].append(cubePos[j])
                #storing their boxid
                dd[k]=botIds[j]
                k+=1
        #box ids
        dictionaries[i].append(dd)

        # calling func to create graph
        if debug:
            for _ in range(3):
                print("")
            print(f"------------- %%creating cluster #{i+1} %% -------------")

		#store parent connections
        parent = create_tree(groups[i],dictionaries[i][0])
        parents[i].append(parent)
        
		#add debugging lines
        if debug==True and pClient is not None:
            r = int(np.random.normal(0,0.5,1)*255)
            g = int(np.random.normal(0,0.8,1)*255)
            b = int(np.random.normal(0,0.7,1)*255)
            for o in range(len(parent)):
                if parent[o]!=-1:
                    p.addUserDebugLine(groups[i][o],groups[i][parent[o]], [r,g,b], 2, physicsClientId = pClient)
    return parents, groups, dictionaries


def sim(num_bots=8, seed=0, num_clusters=2, debug=False):
    """
    Spawn blocks and call clustering related functions

    Parameters:
    num_bots : number of bots to spawn
    seed : random seed
    num_clusters : number of clusters to group data into
    debug : bool - help in debugging and visualisation
    """
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
    area = 1.5
    cubeStartPos = 2*area*(np.random.random((num_bots,2)) - 0.5)
    cubeStartPos = np.append(cubeStartPos, -0.01*np.ones((num_bots,1)), axis=1)
    if debug : print("cube poses ",cubeStartPos)

    #save all boxes
    boxId = []
    #spawn boxes
    for i in range(num_bots):
        boxId.append(p.loadURDF("../iota/absolute/iota.urdf",cubeStartPos[i], cubeStartOrientation))
    #simulate
    i = 0
    while True:
        p.stepSimulation()

        if i%1000==0:
            cubePos = []
            for i in range(num_bots):
                pos, _ = p.getBasePositionAndOrientation(boxId[i])
                cubePos.append(pos)
            cluster(cubePos, boxId, num_clusters, debug)
        i+=1
        time.sleep(1./240.)

    #close connection
