import numpy as np
import pybullet as p
import time
import math
import pybullet_data
from cubic_spline import Spline2D
from iOTA import iOTA
rnd = np.random.random
physicsClient = p.connect(p.GUI)

p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-10)

'''
class iOTA():
    arena_x = 1.5
    arena_y = 1.5
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
'''


def neighbour(X,i,j,v):
  '''
  This function simply marks v in lookup table X,
  for the neighbourhood of (i,j)
  convolution of (3,3) on (i,j) with sum of v

  args:
    VectorFieldLookupTable, i_instance, j_instance, value
  returns:
    None
  '''
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

def second_neighbour(X,i,j,v):
  '''
  Same as above but
  convolution of (5,5) on (i,j) with sum of v

  args:
    VectorFieldLookupTable, i_instance, j_instance, value
  returns:
    None
  '''
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



def list_n(V,i,j,pre):
    '''
    It plans in the neighbourhood to select the neighbouring vertice with minimum value of potential
    args:
        PotentialLookUpTable, i_instance, j_instance, previous_vertex(parent)
    returns:
        NextVertex
    '''
    ## a_dash contains feasible next
    if(i>0 and j>0):
        a_dash = [(i,j+1), (i,j-1), (i+1,j+1), (i-1,j+1), (i+1,j-1), (i-1,j-1), (i-1,j), (i+1,j)]
    elif(i>0 and j==0):
        a_dash = [(i,j+1), (i+1,j+1), (i-1,j+1), (i-1,j), (i+1,j)]
    elif(i==0 and j>0):
        a_dash = [(i,j+1), (i,j-1), (i+1,j+1), (i+1,j-1), (i+1,j)]
    else:
        a_dash = [(i,j+1) , (i+1,j+1), (i+1,j)]

    a_dash.remove(pre) ## of all the vertices removing the parent vertice
    a=[[0] for i in range(len(a_dash))]
    for index in range(len(a_dash)):
        a[index] = V[a_dash[index][0]][a_dash[index][1]] ##choosing the next vertice based on minimum potential value
    minposition = a.index(min(a))
    loc = a_dash[minposition]
    return loc

def quaternion_to_euler(x, y, z, w):
    '''
    Simple conversion of quaternion to euler
    '''
    import math
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    X = math.degrees(math.atan2(t0, t1))

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    Y = math.degrees(math.asin(t2))
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    Z = math.degrees(math.atan2(t3, t4))
    return X, Y, Z

## Spawining plane, dabba, and self and other cars as obstacle.
planeID = p.loadURDF("plane.urdf")
dabba = p.loadURDF("dot.urdf",basePosition=[0,1.5,0])
car = iOTA("../iota/absolute/iota.urdf",physicsClient=physicsClient,arena=[1,1])
min_pos = [-5,-5,0]       ## A grid world setup with corner
max_pos = [5,5,0]         ## A grid world setup with corner
target_pos = p.getBasePositionAndOrientation(dabba)[0]  ## SETPOINT
iotas = [ iOTA("../iota/absolute/iota.urdf", physicsClient=physicsClient, arena=[1,1]) for i in range(30) ]

base_pos=car.get_pos()[0]    ## Self location
particle_optim = [ list(p.getBasePositionAndOrientation(iota.id)[0]) for iota in iotas ]    ## Location of all the obstacles

print("\n"+ 5*"=" + "SPAWNED ALL BOTS" + 5*"="+"\n")

t = time.time()

rto = 5

## Getting the coordinates in grid world coordinates i.e., Indices of cell
[i_base,j_base]= [2*int(round(rto*(base_pos[0]-min_pos[0]))),2*int(round(rto*(base_pos[1]-min_pos[1])))]   ## Multiplied with 10 to increase the resolution of the grid world
[i_max,j_max]= [2*int(round(rto*(max_pos[0]-min_pos[0]))),2*int(round(rto*(max_pos[1]-min_pos[1])))]
[i_target,j_target]=[2*int(round(rto*(target_pos[0]-min_pos[0]))), 2*int(round(rto*(target_pos[1]-min_pos[1])))]

## Initializing lookup tables
D = np.array([[0 for j in range(j_max+1)] for i in range(i_max+1)])     ## positive Potential due to target
B = np.array([[0 for j in range(j_max+1)] for i in range(i_max+1)])     ## monotonic potential due to obstacles
Final = np.array([[0 for j in range(j_max+1)] for i in range(i_max+1)])

for i in range(30):
    ## Doing the same conversion into indicies for all the obstacles
    ii = 2*int(round(rto*(particle_optim[i][0]-min_pos[0])))
    jj =  2*int(round(rto*(particle_optim[i][1]-min_pos[1])))
   # print(ii,jj)
    if(ii>=0 and jj>=0):## If valid point as min_pos is used
       B[ii][jj]=150
       neighbour(B,ii,jj,100) ## Check this.
       #second_neighbour(B,ii,jj,50)



D[i_target][j_target]=2
index_i = i_target
v=2

## Each while loop to facilitate a corner of the grid world

while index_i>=0:
   index_j = j_target
   while index_j >=0:
         neighbour(D,index_i,index_j,(D[index_i][index_j]+1))
         ## marking higher potential for neighbourhood
         ## the gradient is +1 per cell in all directions
         ## loop runs from i_target, j_target to (0,0)
         index_j=index_j-1
   index_i=index_i-1

index_i =i_target

while index_i<i_max:
   index_j =j_target
   while index_j <j_max:
         neighbour(D,index_i,index_j,(D[index_i][index_j]+1))
         ## marking higher potential for neighbourhood
         ## the gradient is +1 per cell in all directions
         ## loop runs from i_target, j_target to (i_max,j_max)
         index_j=index_j+1
   index_i=index_i+1

index_i =i_target

while index_i<i_max:
   index_j =j_target
   while index_j >=0:
         neighbour(D,index_i,index_j,(D[index_i][index_j]+1))
         ## marking higher potential for neighbourhood
         ## the gradient is 1 per cell in all directions
         ## loop runs from i_target, j_target to (i_max,0)
         index_j=index_j-1
   index_i=index_i+1

index_i =i_target

while index_i>=0:
   index_j =j_target
   while index_j <j_max:
         neighbour(D,index_i,index_j,(D[index_i][index_j]+1))
         ## marking higher potential for neighbourhood
         ## the gradient is 1 per cell in all directions
         ## loop runs from i_target, j_target to (0,j_max)
         index_j=index_j+1
   index_i=index_i-1

## Setting the target potential to the minimum
D[i_target][j_target]=-150


for run_i in range(i_max+1):
  for run_j in range(j_max+1):
    Final[run_i][run_j] = B[run_i][run_j] +D[run_i][run_j]
    ## super-imposing the two Potential fields


run_i = i_base
run_j = j_base

#print (run_i,run_j)
k=0
path=[]
pre=(i_base,j_base-1)
x, y = [], []
while (run_i!=i_target or run_j!=j_target):
  pos = list_n(Final,run_i,run_j,pre)
  #print(pos)

  pre = (run_i,run_j)
  run_i=pos[0]
  run_j=pos[1]
  i_p = (0.5/rto*pos[0]+min_pos[0])
  j_p =(0.5/rto*pos[1]+min_pos[1])
  k_p = 0.01
  #print(i_p,j_p)
  x.append(i_p)
  y.append(j_p)
  #path.append([i_p,j_p])
  #pathway = p.loadURDF("dot.urdf",basePosition=[i_p,j_p,k_p])
  #k=k+1

print("\nTime Elapsed: ",time.time()-t,"\n")
ds = 0.05
sp = Spline2D(x, y)
import matplotlib.pyplot as plt
s = np.arange(0, sp.s[-1], ds)

rx, ry = [], []
for i_s in s:
    ix, iy = sp.calc_position(i_s)
    rx.append(ix)
    ry.append(iy)
plt.subplots(1)
plt.plot(x, y, "xb", label="input")
plt.plot(rx, ry, "-r", label="spline")
plt.grid(True)
plt.axis("equal")
plt.xlabel("x[m]")
plt.ylabel("y[m]")
plt.legend()
plt.show()

## Implementation Notes
i = 0
ds = 0.05
sp = Spline2D(x,y)
s = np.arange(0,sp.s[-1], ds)
rx ,ry = [],[]
for i_s in s:
    ix, iy = sp.calc_position(i_s)
    rx.append(ix)
    ry.append(iy)
rx = np.array(rx).reshape(-1,1)
ry = np.array(ry).reshape(-1,1)
vecx = np.diff(rx, axis=0)
vecy = np.diff(ry, axis=0)
while True:
    bp =car.get_pos()[0]

    while ((rx[i]-bp[0])**2+(ry[i]-bp[1])**2) > 0.05:
        car.control((100 * (rx[i][0]-bp[0]) , 100*(ry[i][0]-bp[1]) ))
        p.stepSimulation()
        bp = car.get_pos()[0]
    i+=1
    print(rx[i],ry[i])
time.sleep(0.05)
