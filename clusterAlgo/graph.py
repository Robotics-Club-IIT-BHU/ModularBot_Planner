import random
# random.seed(17)

#number of nodes
nodes = 5

#adjacency matrix 
graph = []

#dummy variables
for i in range(nodes):
	row = [0 for _ in range(nodes)]
	graph.append(row)

#unweighted graph
for _ in range(10):
	p = random.randint(0,nodes-1)
	c = random.randint(0,nodes-1)

	graph[p][c] = 1

print(graph)

#depth first search
start = random.randint(0,nodes-1)

visited = [0 for _ in range(nodes)]

def node_atch(arr):
	for i,ele in enumerate(arr):
		if visited[i] == 0 and ele == 1 :
			print(i)
			visited[i]=1
			node_atch(graph[i])

print(start)
visited[start] = 1
node_atch(graph[start])


