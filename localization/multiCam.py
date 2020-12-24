import pybullet as p
import pybullet_data
import numpy as np
from time import time, sleep
import cv2

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

camView = p.computeViewMatrix(cameraEyePosition = [0,0,1.2],
                              cameraTargetPosition = [0,0,0],
                              cameraUpVector = [0,1,0],
                              physicsClientId = pClient
                              )
camProj = p.computeProjectionMatrixFOV(fov = 90,
                                       aspect = 1.778,
                                       nearVal = 0.1,
                                       farVal = 6,
                                       physicsClientId = pClient
                                       )
cam = lambda : p.getCameraImage(width = HEIGHT,
                              height = WIDTH,
                              viewMatrix = camView,
                              projectionMatrix = camProj,
                              renderer = p.ER_BULLET_HARDWARE_OPENGL,
                              physicsClientId = pClient
                              ) [2]
while True:
    p.stepSimulation()
    img = cam()
    cv2.imshow('frame',img[:,:,[2,1,0]])
    cv2.waitKey(1)

    sleep(0.01)
