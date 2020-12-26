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
basePosition=[(rnd()-0.5)/0.5,
              (rnd()-0.5)/0.5,
              0.05
              ]
iota = p.loadURDF('../iota/absolute/iota.urdf',
                  basePosition = basePosition )
for i in range(p.getNumJoints(iota)):
    print(p.getJointInfo(iota,i))
print("basePosition",basePosition)
cams = []
no_of_cams = 4
cam_poses = [[1,1,1.2], [-1,1,1.2], [-1,-1,1.2], [1,-1,1.2]]
target_pos = [0, 0, 0]
for i in range(no_of_cams):
    per = [0,0,0]
    for j in range(3): per[j] = target_pos[j] - cam_poses[i][j]
    per[2] += 2*cam_poses[i][2]
    print('perpendicular'+str(i), per)
    cam = pybullet_Camera(pos = cam_poses[i],
                           target_pos = target_pos,
                           up_vec = per,
                           width=WIDTH,
                           height=HEIGHT,
                           pClient = pClient,
                           frame_rate = 5,
                           sync = True,
                           sleep_rate = 1             ### only for simulator
                                        #USe only and aonly if the camera is rarely used
                          )
    cams.append(cam)
cameraMatrix = np.load('calibration.npz')['leftCameraMatrix']
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
thresh = 250
while True:
    p.stepSimulation()
    for i, cam in enumerate(cams):
        poses = []
        img = cam.read()
        hsv = cv2.cvtColor(img[:,:,:3], cv2.COLOR_RGB2HSV)
        img = cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)
        mask = cv2.inRange(hsv, LOW_BLUE, HIGH_BLUE)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        can = cv2.Canny(gray, thresh, thresh*2)
        pos = get_pos(can, camera_Rot=Rt_vec[i], cameraMatrix=cameraMatrix,draw=True,img=img)
        poses.append(pos)
        #cv2.imshow('frame'+str(i),img[:,:,[2,1,0]])
        #cv2.imshow('can'+str(i),can)
        cv2.imshow('frame'+str(i), img)
    pose = np.array(poses).mean(axis=0).reshape(-1)
    real_pose = p.getBasePositionAndOrientation(iota)(0) ### The pose outputed is of the baseplates edge and not of the center of mass.
    err = sum(abs(real_pose[i]-pose[i]) for i in range(3
    print(err)          ### This metrics is somewhat representative of the performance
    cv2.waitKey(1)

    sleep(0.01)
