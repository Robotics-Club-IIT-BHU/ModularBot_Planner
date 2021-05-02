import numpy as np
import time
import math
from multiprocessing import Pool
from gevent import Timeout
from gevent import monkey
monkey.patch_all()

def is_inside(i,j,i_max,j_max):
    '''
    This simply checks if the index (i,j) is in side the grid

    args:
        index_i, index_j, i_max, j_max
    returns:
        True if inside the grid else False
    '''

    if i_max>i>0 and j_max>j>0:
        return True
    else:
        return False

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

    for di in range(-1,2,1):
        for dj in range(-1,2,1):
            if is_inside(i+di,j+dj,X.shape[0]-2,X.shape[1]-2) and (i+di, j+dj) != (i, j):
                if X[i+di][j+dj] == 0:
                    X[i+di][j+dj] = v
    return None

def second_neighbour(X,i,j,v):
    '''
    Same as above but
    convolution of (5,5) on (i,j) with sum of v

    args:
      VectorFieldLookupTable, i_instance, j_instance, value
    returns:
      None
    '''

    for di in range(-2,3,1):
        for dj in range(-2,3,1):
            if is_inside(i+di, j+dj,X.shape[0]-2,X.shape[1]-2) and (i+di, j+dj) != (i, j):
                if X[i+di][j+dj] == 0:
                    X[i+di][j+dj] = v
    return None

def list_n(V,i,j,pre):
    '''
    It plans in the neighbourhood to select the neighbouring vertice with minimum value of potential
    args:
        PotentialLookUpTable, i_instance, j_instance, previous_vertex(parent)
    returns:
        NextVertex
    '''

    a_dash = []
    for di in range(-1,2,1):
        for dj in range(-1,2,1):
            if is_inside(i+di, j+dj, V.shape[0]-2,V.shape[1]-2) and (i+di, j+dj) != (i, j):
                if (i+di, j+dj) != pre:
                    a_dash.append((i+di, j+dj))
    
    return min(a_dash, key = lambda x: V[x[0]][x[1]])

def planning_feedforward(base_pos, target_pos, B, ratio, min_pos, max_pos):
    '''
    This computes the path for the given obstacle lookup table this is independent for each bot.
    args:
        Base Position, Target Position, Lookup table of obstacles, Ratio for resolution of the grid world, Minimum position, Maximum position
    returns:
        X vector of coordinates, Y vector of coordinates for the base and target position
    '''    
    [i_base,j_base]= [2*int(round(ratio*(base_pos[0]-min_pos[0]))),2*int(round(ratio*(base_pos[1]-min_pos[1])))]   ## Multiplied with 10 to increase the resolution of the grid world
    [i_max,j_max]= [2*int(round(ratio*(max_pos[0]-min_pos[0]))),2*int(round(ratio*(max_pos[1]-min_pos[1])))]
    [i_target,j_target]=[2*int(round(ratio*(target_pos[0]-min_pos[0]))), 2*int(round(ratio*(target_pos[1]-min_pos[1])))]
    D = np.array([[0 for j in range(j_max+2)] for i in range(i_max+2)])     ## positive Potential due to target
    Final = np.array([[0 for j in range(j_max+2)] for i in range(i_max+2)])
    print("target_pose", target_pos, "base_pose", base_pos, "min_pose", min_pos, "max_pose", max_pos)
    ## Setting a low potential for the target.
    D[i_target][j_target]=2
    index_i = i_target
    v=2

    ## Each while loop to facilitate a corner of the grid world
    ## marking higher potential for neighbourhood
    ## the gradient is +1 per cell in all directions
    while index_i>=0:
       index_j = j_target
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
    
    ## Setting the target potential to the minimum
    D[i_target][j_target]=-150

    for run_i in range(i_max+1):
        for run_j in range(j_max+1):
            Final[run_i][run_j] = B[run_i][run_j] +D[run_i][run_j]
            ## super-imposing the two Potential fields
    run_i = i_base
    run_j = j_base
    pre=(i_base,j_base-1)
    x, y = [], []
    cnt = 0
    print("starting search")
    while cnt<=1000 and (run_i!=i_target or run_j!=j_target):
        pos = list_n(Final,run_i,run_j,pre)
        pre = (run_i, run_j)
        run_i=pos[0]
        run_j=pos[1]
        i_p = (0.5/ratio*pos[0]+min_pos[0])
        j_p =(0.5/ratio*pos[1]+min_pos[1])
        k_p = 0.01
        x.append(i_p)
        y.append(j_p)
        cnt += 1
    print("done")
    return x,y


def wrapper(args):
    '''
    Helper function to unpack multiple arguements for the Pool.map
    '''
    path = ([],[])
    try:
        #tout = Timeout(5)
        #tout.start()
        path = planning_feedforward(*args)
    except:
        path = ([],[])
    finally:
        return path

def planning(base_poses, target_poses, centroid, obstacles, ratio,debug=False):
    '''
    This does the planning using Potential Field Algorithm
    args:
        Base Positions, Target Positions, Positions of obstacles, Ratio for resolution, Debug flag to display additional info
    returns:
        X vector of coordinates, Y vector of coordinates for each base and target positions
    '''

    t = time.time()

    min_pos = [
                min(np.array(obstacles)[:,0].min(), np.array(target_poses)[:,0].min())-0.2,
                min(np.array(obstacles)[:,1].min(), np.array(target_poses)[:,1].min())-0.2,
                0
                ] ## Needs to be changed
    max_pos = [
                max(np.array(obstacles)[:,0].max(), np.array(target_poses)[:,0].max())+0.2,
                max(np.array(obstacles)[:,1].max(), np.array(target_poses)[:,1].max())+0.2,
                0] ## Needs to be changed

    ## Getting the coordinates in grid world coordinates i.e., Indices of cell
    #[i_base,j_base]= [2*int(round(ratio*(base_pos[0]-min_pos[0]))),2*int(round(ratio*(base_pos[1]-min_pos[1])))]   ## Multiplied with 10 to increase the resolution of the grid world
    [i_max,j_max]= [2*int(round(ratio*(max_pos[0]-min_pos[0]))),2*int(round(ratio*(max_pos[1]-min_pos[1])))]
    #[i_target,j_target]=[2*int(round(ratio*(target_pos[0]-min_pos[0]))), 2*int(round(ratio*(target_pos[1]-min_pos[1])))]

    ## Initializing lookup tables
    B = np.array([[0 for j in range(j_max+1)] for i in range(i_max+1)])     ## monotonic potential due to obstacles
    Final = np.array([[0 for j in range(j_max+1)] for i in range(i_max+1)])

    for obstacle in obstacles:
        ## Doing the same conversion into indicies for all the obstacles
        ii = 2*int(round(ratio*(obstacle[0]-min_pos[0])))
        jj = 2*int(round(ratio*(obstacle[1]-min_pos[1])))
        if(ii>=0 and jj>=0): ## If valid point as min_pos is used
           B[ii][jj]=150
           neighbour(B,ii,jj,100)
    print("starting")
    ## Pooling the computation as they are independent to each other
    #with Pool(1) as p:
    #    Paths = p.map(wrapper, [(base_pos, target_pos, B.copy(), ratio, min_pos, max_pos) for base_pos, target_pos in zip(base_poses, target_poses) ] )
    Paths = []
    for base_pos, target_pos in zip(base_poses, target_poses):
        Paths.append(wrapper((base_pos, target_pos, B.copy(), ratio, min_pos, max_pos)))
    print("done Planning")
    if debug :
        print(time.time()-t)
    return Paths
