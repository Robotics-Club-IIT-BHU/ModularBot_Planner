import numpy as np
import pybullet as p
import time
import math
import pybullet_data
rnd = np.random.random
physicsClient = p.connect(p.GUI)

p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-50)

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

      return loc

def quaternion_to_euler(x, y, z, w):

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

planeID = p.loadURDF("plane.urdf")
dabba = p.loadURDF("dot.urdf",basePosition=[10,12,0])
car = p.loadURDF("iota.urdf")
min_pos = [-20,-20,0]
max_pos = [20,20,0]
target_pos = p.getBasePositionAndOrientation(dabba)[0]
iotas = [ iOTA("dabba.urdf", physicsClient=physicsClient) for i in range(25) ]

base_pos=p.getBasePositionAndOrientation(car)[0]
particle_optim = [ list(p.getBasePositionAndOrientation(iota.id)[0]) for iota in iotas ]

#print(base_pos)
#print(particle_optim)
[i_base,j_base]= [2*int(round(base_pos[0]-min_pos[0])),2*int(round(base_pos[1]-min_pos[1]))]
[i_max,j_max]= [2*int(round(max_pos[0]-min_pos[0])),2*int(round(max_pos[1]-min_pos[1]))]
[i_target,j_target]=[2*int(round(target_pos[0]-min_pos[0])), 2*int(round(target_pos[1]-min_pos[1]))]
#print(i_target,j_target)

D=[[0] * (j_max+1) for i in range(i_max+1)]

B=[[0] * (j_max+1) for i in range(i_max+1)]
Final=[[0] * (j_max+1) for i in range(i_max+1)]
for i in range(25):
    ii = 2*int(round(particle_optim[i][0]-min_pos[0]))
    jj =  2*int(round(particle_optim[i][1]-min_pos[1]))
   # print(ii,jj)
    if(ii>=0 and jj>=0):
       B[ii][jj]=150
       neighbour(B,ii,jj,100)
       #second_neighbour(B,ii,jj,50)
    


D[i_target][j_target]=2
index_i =i_target
v=2
#for row in D:
  #  print(' '.join([str(elem) for elem in row]))

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


for run_i in range(i_max+1):
  for run_j in range(j_max+1):
    Final[run_i][run_j] = B[run_i][run_j] +D[run_i][run_j]

#for row in Final:
  #  print(' '.join([str(elem) for elem in row]))

run_i = i_base
run_j = j_base

#print (run_i,run_j)
k=0
path=[]
pre=(i_base,j_base-1)
while (run_i!=i_target or run_j!=j_target):
  pos = list_n(Final,run_i,run_j,pre)
  #print(pos)
  
  pre = (run_i,run_j)  
  run_i=pos[0]
  run_j=pos[1]
  i_p = (0.5*pos[0]+min_pos[0])
  j_p =(0.5*pos[1]+min_pos[1])
  k_p = 0.01
  print(i_p,j_p)
  path.append([i_p,j_p])
  pathway = p.loadURDF("random.urdf",basePosition=[i_p,j_p,k_p])
  #k=k+1
  
while(1):
   p.stepSimulation()
time.sleep(0.05)
