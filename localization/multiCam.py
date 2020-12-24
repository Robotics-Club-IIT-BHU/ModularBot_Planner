import pybullet as p
import pybullet_data
import numpy as np
from time import time, sleep
import cv2
from cameraSetup import pybullet_Camera
if p.isNumpyEnabled(): print('Numpy enabled cooolll!!.')

WIDTH = 360
HEIGHT = 640
rnd = np.random.random

pClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadURDF('plane.urdf')
p.setGravity(0, 0, -10)

iota = p.loadURDF('../iota/absolute/iota.urdf',
                  basePosition = [(rnd()-0.5)/0.5,
                                (rnd()-0.5)/0.5,
                                0.001
                                ])
cams = []
no_of_cams = 4
cam_poses = [[1,1,1.2], [-1,1,1.2], [-1,-1,1.2], [1,-1,1.2]]
target_pos = [0, 0, 0]
for i in range(no_of_cams):
    per = [0,0,0]
    for j in range(3): per[j] = target_pos[j] - cam_poses[i][j]
    per[2] += 2.4
    cam = pybullet_Camera(pos = cam_poses[i],
                           target_pos=target_pos,
                           up_vec=per,
                           pClient=pClient,
                           frame_rate=15,
                           sleep_rate=1             ### only for simulator
                          )
    cams.append(cam)
for cam in cams:
    cam.connect()
while True:
    p.stepSimulation()
    for i, cam in enumerate(cams):
        img = cam.read()
        cv2.imshow('frame'+str(i),img[:,:,[2,1,0]])
    cv2.waitKey(1)

    sleep(0.01)
