import numpy as np
import pybullet as p
import time
import math

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
            if is_inside(i+di,j+dj) and (i+di, j+dj) != (i, j):
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
            if is_inside(i+di, j+dj) and (i+di, j+dj) != (i, j):
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
            if is_inside(i+di, j+dj) and (i+di, j+dj) != (i, j):
                if (i+di, j+dj) != pre:
                    a_dash.append((i+di, j+dj))

    return min(a_dash, key = lambda x: V[x[0]][x[1]])
