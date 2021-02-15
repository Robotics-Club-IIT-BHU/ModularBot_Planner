import gym
import pybullet as p
import gym_iOTA
import numpy as np
import time
from clustering import cluster
from planning import *
from experimental import ParamPoly2D
env = gym.make('iOTA-v0',
                render=True,
                no_of_modules=10,
                no_of_clusters=2,
                arena=(3,3),
                low_control=False,
                )

observation = env.reset()
i = 0
poly = ParamPoly2D(2,10)
#print(poly.root_coor)
setpoints = np.zeros((env.no_of_modules, 3))
for j in range(env.no_of_modules):
    setpoints[j,:] = [*poly.sample(observation[j,:]),0.01] 
#print(observation, setpoints)
paths = planning(observation, setpoints, (0,0,0), observation, 5)
print("planning done")
smooth_path = []
progress = []
ds = 0.05
for path in paths:
    sp = Spline2D(*path)
    s = np.arange(0, sp.s[-1], ds)
    rx, ry = [], []
    for i_s in s:
        ix, iy = sp.calc_position(i_s)
        rx.append(ix)
        ry.append(iy)
    smooth_path.append(list(zip(rx,ry)))
    progress.append(0)
while i>=0: 
    action = np.ones((env.no_of_modules, 3))
    for j in range(env.no_of_modules):
        if progress[j]!=-1:
            if progress[j]==(len(smooth_path[j])-1):
                progress[j] = -1
            else: 
                progress[j] +=1
        action[j,:] = [*smooth_path[j][progress[j]], 0.01]
    dock = np.zeros(
              (env.no_of_modules,
              env.no_of_modules))             
    observation, reward, done, info = env.step(action, dock)
    ## Try pooling the control
    if i%1000==0:
        for j in range(env.no_of_modules):
            setpoints[j,:] = [*poly.sample(observation[j,:2]),0.01] 
        cluster(observation, [iota.id for iota in env.iotas], env.no_of_clusters, debug=True, pClient=env.pClient )
    time.sleep(0.1)
    i+=1
env.close()