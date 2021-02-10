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
                arena=(2,2),
                low_control=True,
                )

observation = env.reset()
i = 0
poly = ParamPoly2D(5,1)
setpoints = np.zeros((env.no_of_modules, 3))
for j in range(env.no_of_modules):
    setpoints[j.:] = [*poly.sample(observation[j,:2]),0.01] 

while i>=0:
    action = np.ones((env.no_of_modules, 4))   
    dock = np.zeros(
              (env.no_of_modules,
              env.no_of_modules))             
    observation, reward, done, info = env.step(action, dock)
    if i%100==0:
        for j in range(env.no_of_modules):
            setpoints[j.:] = [*poly.sample(observation[j,:2]),0.01] 
        cluster(observation, [iota.id for iota in env.iotas], env.no_of_clusters, debug=True, pClient=env.pClient )
    time.sleep(0.1)
    i+=1
env.close()