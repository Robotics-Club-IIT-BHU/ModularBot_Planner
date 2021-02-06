import numpy as np
import pybullet as p
import time
import math
import pybullet_data
from iOTA import iOTA
from potential_field import planning
from cubic_spline import Spline2D
rnd = np.random.random
physicsClient = p.connect(p.GUI)

p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-10)

## Spawining plane, dabba, and self and other cars as obstacle.
planeID = p.loadURDF("plane.urdf")
##dabba = p.loadURDF("dot.urdf",basePosition=[0,1.5,0])
##car = p.loadURDF("../iota/absolute/iota.urdf")
iotas = [ iOTA("../iota/absolute/iota.urdf", physicsClient=physicsClient, arena=[1.5,1.5]) for i in range(10) ]
ratio = 5
#base_pos=p.getBasePositionAndOrientation(car)[0]    ## Self location
particle_optim = [ list(p.getBasePositionAndOrientation(iota.id)[0]) for iota in iotas ]    ## Location of all the obstacles

target_poses = [ [1,0, 0.01]  for i in range(10) ]

paths = planning(particle_optim, target_poses, [0,0,0.01], particle_optim, ratio, False)
ds = 0.05
sp1 = Spline2D(*paths[0])
sp2 = Spline2D(*paths[5])
import matplotlib.pyplot as plt
s1 = np.arange(0, sp1.s[-1], ds)
s2 = np.arange(0, sp2.s[-1], ds)
rx1, ry1 = [], []
rx2, ry2 = [], []
for i_s in s1:
  ix, iy = sp1.calc_position(i_s)
  rx1.append(ix)
  ry1.append(iy)
for i_s in s2:
  ix, iy = sp2.calc_position(i_s)
  rx2.append(ix)
  ry2.append(iy)

plt.subplots(1)
plt.plot(*paths[0], "xb", label="input")
plt.plot(rx1, ry1, "-r", label="spline")
plt.grid(True)
plt.axis("equal")
plt.xlabel("x[m]")
plt.ylabel("y[m]")
plt.legend()

plt.subplots(1)
plt.plot(*paths[5], "xb", label="input")
plt.plot(rx2, ry2, "-r", label="spline")
plt.grid(True)
plt.axis("equal")
plt.xlabel("x[m]")
plt.ylabel("y[m]")
plt.legend()

plt.show()

while True:
  p.stepSimulation()
  time.sleep(0.05)
time.sleep(0.05)
