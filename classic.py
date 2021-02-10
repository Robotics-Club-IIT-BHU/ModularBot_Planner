import gym
import pybullet as p
import gym_iOTA
import numpy as np
import time
from clustering import cluster
env = gym.make('iOTA-v0',
                render=True,
                no_of_modules=10,
                no_of_clusters=2,
                arena=(2,2),
                )

env.reset()
while True:
    action = np.ones((env.no_of_modules, 4))   
    dock = np.zeros(
              (env.no_of_modules,
              env.no_of_modules))             

    observation, reward, done, info = env.step(action, dock)
    cluster(observation, [iota.id for iota in env.iotas], env.no_of_clusters, debug=True, pClient=env.pClient )
    time.sleep(0.1)
env.close()