import sys
import numpy as np

class Graph:
	def __init__(self,vertices, dicts):
		self.vertices = vertices
		self.graph = [ [0.0 for _ in range(vertices)] for _ in range(vertices)]
		self.graph = np.array(self.graph)
		self.dicts = dicts


	def makeEdge(self,v1,v2,weight):
		"""
		Create any edge between two points

		Parameters:
		v1 : vertice one
		v2 : vertice two
		weight : edge weight
		"""

		self.graph[v1][v2] = weight;
		self.graph[v2][v1] = weight;


	def printGraph(self):
		"""
		Print graph
		"""
		print(self.graph)


	def printTree(self,parent):
		"""
		Print each node's parent in mst

		Parameter:
		parent : array of vertices of parents
		"""
		print('parent 			child')
		for v in range(self.vertices):
			if parent[v]==-1:
				continue
			print(f'{parent[v]} boxid:{self.dicts[parent[v]]} - {v} boxid:{self.dicts[v]} (weight {self.graph[v][parent[v]]})')

	def minKey(self,mstSet,key):
		"""
		Find the vertex with least key value not included in mstSet

		Parameter:
		mstSet : set of visited or traversed nodes
		key : list of distances or weights between nodes

		Return:
		min_index : list of index closest to given node
		"""

		min = sys.maxsize
		min_index = None

		for i in range(self.vertices):
			if key[i]<min and mstSet[i] == False:
				min = key[i]
				min_index = i

		return min_index 

	def mnst(self,head):
		"""
		Build the minimum spanning tree

		Parameter:
		head : starting node index

		Return:
		parent : list of indices of parent of nodes
		"""

		#creating the key list
		key = [sys.maxsize]*self.vertices
		key[head] = 0

		#list of vertices already a part of mst
		mstSet = [False]*self.vertices

		#list to store parent
		parent = [None]*self.vertices
		parent[head]=-1 #cluster head has no parent

		for _ in range(self.vertices):
			#finding the vertice with least key
			u = self.minKey(key=key,mstSet=mstSet)

			#set the mstSet
			mstSet[u] = True

			for v in range(self.vertices):
				if mstSet[v] == False and self.graph[u][v]>0 and key[v]>self.graph[u][v]:
					key[v] = self.graph[u][v]
					parent[v]=u

		# self.printTree(parent)
		return parent

if __name__=='__main__':
	#define cluster head here
	head = 1

	g = Graph(5)
	g.graph = [ [0, 2, 0, 7, 0],
	            [2, 0, 3, 8, 5],
	            [0, 3, 0, 0, 7],
	            [7, 8, 0, 0, 9],
	            [0, 5, 7, 9, 0]]

	g.mnst(head)