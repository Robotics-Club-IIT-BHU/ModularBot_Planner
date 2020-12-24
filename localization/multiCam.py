import pybullet as p
import pybullet_data
import numpy as np
from time import time, sleep
import cv2
from vector import Vect3d, Vect2d
from cameraSetup import pybullet_Camera
from Reconstruction import vec2rotm, get_pos

if p.isNumpyEnabled(): print('Numpy enabled cooolll!!.')

WIDTH = 640
HEIGHT = 360
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
    per[2] += 2*cam_poses[i][2]
    cam = pybullet_Camera(pos = cam_poses[i],
                           target_pos = target_pos,
                           up_vec = per,
                           width=WIDTH,
                           height=HEIGHT,
                           pClient = pClient,
                           frame_rate = 15,
                           sync = True,
                           sleep_rate = 1             ### only for simulator
                                        #USe only and aonly if the camera is rarely used
                          )
    cams.append(cam)
for cam in cams:
    cam.connect()
origin = Vect3d(0,0,0)
Rt_vec = []
for cam in cams:
    t = Vect3d(*cam.self_info['position']) - origin
    point_vec = (Vect3d(*cam.self_info['target_pos']) - Vect3d(*cam.self_info['position'])).my_unit()
    R = vec2rotm(point_vec, Vect3d(*cam.self_info['up_vec']))
    t = np.array([*t.get_values()]).reshape(3,1)
    Rt_vec.append([R,t])
    print(R,'\n', t, "\n")

HIGH_BLUE = (130,205,200)
LOW_BLUE = (100,30,30)
while True:
    p.stepSimulation()
    for i, cam in enumerate(cams):
        img = cam.read()
        hsv = cv2.cvtColor(img[:,:,:3], cv2.COLOR_RGB2HSV)
        img = img[:,:,[2,1,0]]
        mask = cv2.inRange(hsv, LOW_BLUE, HIGH_BLUE)
        get_pos(mask, camera_Rot=np.append(Rt_vec[i][0],Rt_vec[i][1],axis=1), cameraMatrix=None)
        #cv2.imshow('frame'+str(i),img[:,:,[2,1,0]])
        cv2.imshow('frame'+str(i),mask)
    cv2.waitKey(1)

    sleep(0.01)
