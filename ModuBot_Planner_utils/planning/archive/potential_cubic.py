import numpy as np
import pybullet as p
import time
import math
import pybullet_data
rnd = np.random.random
physicsClient = p.connect(p.GUI)

p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-50)


def Path_generator(min_pos,max_pos,target_pos,car,particle_optim,no_obs):
	#instead of long range continuous field, barriers add extra potential at neighbour sites and second nearest neighbour sites
	# inputs are field matrix, x coordinate of site, y coordinate of site , the value of field assigned to neighbours
	def neighbour(X,i,j,v):
		if (j!=j_max and X[i][j+1]==0):
			X[i][j+1]=v
	  	if (j!=0 and X[i][j-1]==0):
			X[i][j-1]=v
	  	if (j!=j_max and i!=i_max and X[i+1][j+1]==0) :
	    	X[i+1][j+1]=v
	  	if (j!=j_max and i!=0 and X[i-1][j+1]==0) :
	    	X[i-1][j+1]=v
	  	if (j!=0 and i!=i_max and X[i+1][j-1]==0) :
	    	X[i+1][j-1]=v
	  	if (j!=0 and i!=0 and X[i-1][j-1]==0) :
	    	X[i-1][j-1]=v
	  	if (i!=0 and X[i-1][j]==0):
	    	X[i-1][j]=v
	  	if (i!= i_max and X[i+1][j]==0) :
	    	X[i+1][j]=v

        # all the if's statements are written to avoid boundary sites
        # inputs are field matrix, x coordinate of site, y coordinate of site , the value of field assigned to second nearest neighbours
	def second_neighbour(X,i,j,v):
	  	if (j<j_max-1 and X[i][j+2]==0):
	    	X[i][j+2]=v
	  	if (j>1 and X[i][j-2]==0):
	    	X[i][j-2]=v
	  	if (j<j_max-1 and i<i_max-1 and X[i+2][j+2]==0) :
	    	X[i+2][j+2]=v
	  	if (j<j_max-1 and i>1 and X[i-2][j+2]==0) :
	    	X[i-2][j+2]=v
	  	if (j>1 and i<i_max-1 and X[i+2][j-2]==0) :
	    	X[i+2][j-2]=v
	  	if (j>1 and i>1 and X[i-2][j-2]==0) :
	    	X[i-2][j-2]=v
	  	if (i>1 and X[i-2][j]==0):
	    	X[i-2][j]=v
	  	if (i< i_max-1 and X[i+2][j]==0) :
	    	X[i+2][j]=v

		if (j<j_max and i<i_max-1 and X[i-2][j+1]==0):
	    	X[i-2][j+1]=v
	  	if (i<i_max-1 and j!=0 and X[i-2][j-1]==0):
	    	X[i-2][j-1]=v
	  	if (j<j_max-1 and i!=i_max and X[i-1][j-2]==0) :
	    	X[i-1][j-2]=v
	  	if (j<j_max-1 and i!=i_max and X[i+1][j-2]==0) :
	    	X[i+1][j-2]=v
	  	if (j!=0 and i<i_max-1 and X[i+2][j-1]==0) :
	    	X[i+2][j-1]=v
	  	if (j!=j_max and i<i_max-1 and X[i+2][j+1]==0) :
	    	X[i+2][j+1]=v
	  	if (i!=i_max and j<j_max-1 and X[i+1][j+2]==0):
	    	X[i+1][j+2]=v
	  	if (i!= 0 and j<j_max-1 and X[i-1][j+2]==0) :
	    	X[i-1][j+2]=v
	# function checks the neighbour site with lowest potential
    # inputs are field matrix, x coord. of site, y coord. of site, previous position
	def list_n(V,i,j,pre):
		if(i>0 and j>0):
			a_dash = [(i,j+1), (i,j-1), (i+1,j+1), (i-1,j+1), (i+1,j-1), (i-1,j-1), (i-1,j), (i+1,j)]
		elif(i>0 and j==0):
			a_dash = [(i,j+1), (i+1,j+1), (i-1,j+1), (i-1,j), (i+1,j)]
		elif(i==0 and j>0):
			a_dash = [(i,j+1), (i,j-1), (i+1,j+1), (i+1,j-1), (i+1,j)]
		else:
			a_dash = [(i,j+1) , (i+1,j+1), (i+1,j)]
		a_dash.remove(pre)
		a=[[0] for i in range(len(a_dash))]
		for index in range(len(a_dash)):
			a[index] = V[a_dash[index][0]][a_dash[index][1]]
		minposition = a.index(min(a))
		loc = a_dash[minposition]
		return loc              #output is next location

        # discretization of space. The smallest step is half units
    base_pos=p.getBasePositionAndOrientation(car)[0]
	[i_base,j_base]= [2*int(round(base_pos[0]-min_pos[0])),2*int(round(base_pos[1]-min_pos[1]))]
    [i_max,j_max]= [2*int(round(max_pos[0]-min_pos[0])),2*int(round(max_pos[1]-min_pos[1]))]
    [i_target,j_target]=[2*int(round(target_pos[0]-min_pos[0])), 2*int(round(target_pos[1]-min_pos[1]))]


    D=[[0] * (j_max+1) for i in range(i_max+1)]        # attractive field matrix
	B=[[0] * (j_max+1) for i in range(i_max+1)]        #repulsive field matrix
	Final=[[0] * (j_max+1) for i in range(i_max+1)]    # total field = D + B


	#assigning potential for barriers and their neighbouring locations
	for i in range(no_obs):
	    ii = 2*int(round(particle_optim[i][0]-min_pos[0]))
	    jj =  2*int(round(particle_optim[i][1]-min_pos[1]))

	    if(ii>=0 and jj>=0):
	       B[ii][jj]=150
	       neighbour(B,ii,jj,100)
	       #second_neighbour(B,ii,jj,50)



	D[i_target][j_target]=2
	index_i =i_target
	v=2

	#  assigning potential at each point of space depending on distance from target
	while index_i>=0:
	   	index_j =j_target
	   	while index_j >=0:
		 	neighbour(D,index_i,index_j,(D[index_i][index_j]+1))
		 	index_j=index_j-1
	   	index_i=index_i-1

	index_i =i_target

	while index_i<i_max:
		index_j =j_target
	   	while index_j <j_max:
		   neighbour(D,index_i,index_j,(D[index_i][index_j]+1))
		 index_j=index_j+1
	   	index_i=index_i+1

	index_i =i_target

	while index_i<i_max:
	   	index_j =j_target
	   	while index_j >=0:
		 	neighbour(D,index_i,index_j,(D[index_i][index_j]+1))
		 	index_j=index_j-1
	   	index_i=index_i+1

	index_i =i_target

	while index_i>=0:
	   	index_j =j_target
	   	while index_j <j_max:
		 	neighbour(D,index_i,index_j,(D[index_i][index_j]+1))
		 	index_j=index_j+1
	   	index_i=index_i-1


	D[i_target][j_target]=-150


	#for row in D:
	  #  print(' '.join([str(elem) for elem in row]))
	#for row in B:
	  #  print(' '.join([str(elem) for elem in row]))

	#adding both potential fields
	for run_i in range(i_max+1):
		for run_j in range(j_max+1):
			Final[run_i][run_j] = B[run_i][run_j] +D[run_i][run_j]

	#for row in Final:
	  #  print(' '.join([str(elem) for elem in row]))

	run_i = i_base
	run_j = j_base

	# from base_pos, finding min potential points to plan the final path
	k=0
	path=[]
	count=0
	pre= (i_base,j_base-1)
	danger=[]
	while (run_i!=i_target or run_j!=j_target):
		pos = list_n(Final,run_i,run_j,pre)
		pre = (run_i,run_j)
		run_i=pos[0]
		run_j=pos[1]
		i_p = (0.5*pos[0]+min_pos[0])
		j_p =(0.5*pos[1]+min_pos[1])
		print(i_p,j_p)
		path.append([i_p,j_p])
		'''
	  #flag=0
	  #for i in range(len(path)-1):
	   #    if path[count]==path[i]:                  # not solved yet. (in case car gets stuck in local minimum potential loop)
		#        danger.append(path[count])
		 #       if danger[-1]==danger[0]:
		  #           flag=1
		   #          print(count,flag)
		  '''
		pathway = p.loadURDF("random.urdf",basePosition=[i_p,j_p,0.01])    # these are small red dots loaded to trace the path
	  	count=count+1
	print(path)
	return path

#to inroduce barriers in random locations
class iOTA():
    arena_x = 10
    arena_y = 14
    def __init__(self, path=None, physicsClient=None):
        if path is None:
            path="urdf/dabba.urdf"
        self.pClient = physicsClient
        self.id = p.loadURDF(path,
                            basePosition=self.init_pos(),
                            physicsClientId=self.pClient)

    def init_pos(self):
        return [self.arena_x*(rnd()-0.5)/0.5,
                self.arena_y*(rnd()-0.5)/0.5,
                0.001
                ]
# cubic spline function....input 'S' is vector of points, output 'xy' is vector of smooth path (10 points are interpolated between points of original vector)
def CubicSpline(S):
 	S_dash= [S[0][0]]
    endpoint= S[-1]
 	for i in range(1,len(S)):
        S_dash.append(S[i][0]-S[i-1][0])             # to avoid multiple y points at single x
    print(S_dash)
    count=0
	for i in range(1,len(S)):
		if S_dash[i]==0:
			S.pop(i-count)
			count=count+1
		S[-1]=endpoint
    print(S)
	n=len(S)
    h = [0 for i in range(n)]
    p = [0 for i in range(n)]
	q = [0 for i in range(n)]
    b = [0 for i in range(n)]
	Par =[[0] * (4) for i in range(n)]
	h[0] = S[1][0]-S[0][0]

	for i in range(1,n-1):
		h[i]= S[i+1][0]-S[i][0]
		p[i]=2*(S[i+1][0]-S[i-1][0])
		q[i]= (3*(S[i+1][1]-S[i][1])/h[i])- (3*(S[i][1]-S[i-1][1])/h[i-1])

	for i in range (2,n-1):
		if (p[i-1]==0):
			p[i-1]=0.0001
		p[i]=p[i]-(h[i-1]*h[i-1]/p[i-1])
		q[i]=q[i]-(q[i-1]*h[i-1]/p[i-1])
	b[n-2] = q[n-2]/p[n-2]
	for i in range( 2,n):
		b[n-i]=(q[n-i]-h[n-i]*b[n-i+1])/p[n-i]

	Par[0][0]=b[1]/(3*h[0])                                 # parameter a for initial point
	Par[0][1]= 0                                            # parameter b for initial point
	Par[0][2]= (S[1][1]-S[0][1])/h[0]-b[1]*h[0]/3           # parameter c for initial point
	Par[0][3]=S[0][1]                                       # parameter d for initial point
	Par[n-1][1]= 0

	for i in range(1,n-1):
		Par[i][0] = (b[i+1]-b[i])/(3*h[i])                                      # parameter a
		Par[i][1] = b[i]	                                                # parameter b
		Par[i][2]= (S[i+1][1]-S[i][1])/h[i]-(2*h[i]*b[i]+h[i]*b[i+1])/3          # parameter c
		Par[i][3] = S[i][1]						       # parameter d

	t = 10 #steps
	New_x =  [[0] * (t) for i in range(n-1)]      #new points for x
	New_y =  [[0] * (t) for i in range(n-1)]       # new points for y
	xy=[]

	for i in range(n-1):
		for j in range(t):
			New_x[i][j] =round( S[i][0]+float((S[i+1][0]-S[i][0])*j)/t ,3)
			New_y[i][j] =round( Par[i][0]*((New_x[i][j]-S[i][0])**3)+Par[i][1]*((New_x[i][j]-S[i][0])**2)+Par[i][2]*(New_x[i][j]-S[i][0])+Par[i][3] ,3)
			xy.append((New_x[i][j],New_y[i][j]))
	return xy

planeID = p.loadURDF("plane.urdf")
dabba = p.loadURDF("dot.urdf",basePosition=[10,12,0])
car = p.loadURDF("iota.urdf")
min_pos = [-20,-20,0]    # max and min locations of space we are accounting to introduce field
max_pos = [20,20,0]
no_obs =70               # no. of obstacles
target_pos = p.getBasePositionAndOrientation(dabba)[0]
iotas = [ iOTA("dabba.urdf", physicsClient=physicsClient) for i in range(no_obs) ]
particle_optim = [ list(p.getBasePositionAndOrientation(iota.id)[0]) for iota in iotas ]

path = Path_generator(min_pos,max_pos,target_pos,car,particle_optim,no_obs)      # path generator function
path_smooth =   CubicSpline(path)                                                # cubic spline function

for i in range(len(path_smooth)):   # these are small red dots loaded to trace the path
	pathway = p.loadURDF("random.urdf",basePosition=[path_smooth[i][0],path_smooth[i][1],0.01])
while(1):
	p.stepSimulation()
time.sleep(0.05)
