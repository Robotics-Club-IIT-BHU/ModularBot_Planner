import pybullet as p
import pybullet_data
import numpy as np
import math
from experimental import ParamPoly2D
from iota import iOTA
import time
pc = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadURDF('plane.urdf')
p.setGravity(0,0,-10)
iota = iOTA(position=[1,1,0.01],physicsClient=pc,arena=[1,1])
poly = ParamPoly2D(1,9)
centroid = [-c for c in iota.get_pos()[0][:2]]
prev = None
for root in zip(*poly.root_coor):
    if prev is None:
        prev = root
        first = prev
        print(root)
    else:
        p.addUserDebugLine([*prev,0.01],[*root,0.01],[0,0.6,0],5,0)
        prev = root
p.addUserDebugLine([*prev,0.01],[*first,0.01],[0,0.6,0],5,0)
n = len(poly.curve[0])
m = 2
mid = 10*m
ans = mid + 4
opp = n//2 + mid
prev = None
for i,point in enumerate(zip(*poly.curve)):
    if i<n//2:
        h = 0.5*np.exp(-((i/10 - m)**2)/(2))
    else :
        h = 0.5*np.exp(-(( (n-i) /10 + m)**2)/(2))
    p.addUserDebugLine([*point,0.01],[*point,h],[1,0.7,0] if ans!=i else [1,0,0],5,0)
    if prev is None:
        prev = [*point,h]
        first = prev
    else:
        p.addUserDebugLine(prev,[*point,h],[1,0.4,0],5,0)
        prev = [*point,h]
p.addUserDebugLine(prev,first,[1,0.4,0],5,0)

while True:
    p.stepSimulation()
    time.sleep(0.01)
p.disconnect()
