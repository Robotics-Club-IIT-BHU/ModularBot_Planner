import pybullet as p
import pybullet_data
from math import sqrt
import numpy as np
rnd = np.random.random
import time
class iOTA():
    arena_x = 5
    arena_y = 5
    l_wheels = [15, 19]
    r_wheels = [17, 21]
    docks = [25, 11]
    def __init__(self, path=None, physicsClient=None):
        if path is None:
            path="../iota/absolute/iota.urdf"
        self.pClient = physicsClient
        self.vel = [6*(rnd()-0.5)/0.5, 6*(rnd()-0.5)/0.5]
        theta = np.arctan(self.vel[1]/self.vel[0]) +np.pi
        if self.vel[0] < 0 and self.vel[1]>0:
            theta += np.pi
        orie = p.getQuaternionFromEuler((0,0,theta))
        self.id = p.loadURDF(path,
                            basePosition=self.init_pos(),
                            baseOrientation=orie,
                            physicsClientId=self.pClient)
        self.dockees = []
        self.constraints = []

    def init_pos(self):
        return [self.arena_x*(rnd()-0.5)/0.5,
                self.arena_y*(rnd()-0.5)/0.5,
                0.001
                ]
    def dist(self, target):
        pos = p.getBasePositionAndOrientation(self.id, physicsClientId=self.pClient)[0]
        return sqrt((target[0]-pos[0])**2 + (target[1]-pos[1])**2)
    def control(self,vel_vec):
        #vel_vec =
        orie = p.getBasePositionAndOrientation(self.id, self.pClient)[1]
        yaw = p.getEulerFromQuaternion(orie)[2]
        set_vec = np.arctan(vel_vec[1]/vel_vec[0]) + np.pi
        if vel_vec[0]<0 and vel_vec[1] > 0 :
            set_vec += np.pi
        r = sqrt(vel_vec[0]**2 + vel_vec[1]**2)
        theta = set_vec - yaw
        for_vec = [np.cos(theta)*r, np.cos(theta)*r]
        rot_vec = [-np.sin(theta)*r, np.sin(theta)*r]
        for i,wheel_set in enumerate([self.r_wheels, self.l_wheels]):
            for wheel in wheel_set:
                p.setJointMotorControl2(self.id,
                                        wheel,
                                        controlMode=p.VELOCITY_CONTROL,
                                        targetVelocity=(for_vec[i]+rot_vec[i]),
                                        force=100,
                                        physicsClientId=self.pClient)
        self.vel = vel_vec
        #p.setJointMotorControl2()
    def dock(self,other):
        self.dockees.append(other)
        other.dockess.append(self)
        diff12 = [0,0,0]
        diff21 = [0,0,0]

        for i in range(3):
            diff12[i] = (pos1[i] - pos2[i])/2
            diff21[i] = (pos2[i] - pos1[i])/2

        cid = p.createConstraint(iota1,
                                -1,                 ## front docking plate
                                iota2,
                                -1,                 ## back docking plate
                                p.JOINT_FIXED,
                                [0,0,0],
                                diff21,
                                diff12,
                                self.pClient)
        self.constraints.append(cid)
        other.constraints.append(cid)
        return cid
    def undock(self,other):
        ind = None
        try:
            ind = self.dockees.index(other)
        except:
            raise Exception("These two modules have not been docked before")
            # print(self.id, other.id, "Are not docked before")
        finally:
            if ind is not None:
                cid = self.constraints.pop(ind)
                other.constraints.remove(cid)
                p.removeConstraint(cid,self.pClient)
                self.dockees.remove(other)
                other.dockees.remove(self)
    def __add__(self,other):
        return self.dock(other)

    def __sub__(self,other):
        self.undock(other)
        return -1


def distance(pos, target):
    return ((pos[0]-target[0])**2+(pos[1]-target[1])**2)

pClient = p.connect(p.GUI)


p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-10)
p.loadURDF("plane.urdf")

carpos = [0,0,0.01]
carpos2 = [0.4,0,0.01]
carpos3 = [0.8,0,0.01]
carpos4 = [1.2,0,0.01]
iota1 = p.loadURDF("../iota/absolute/iota.urdf",*carpos)
iota2 = p.loadURDF("../iota/absolute/iota.urdf",*carpos2)
#iota3 = p.loadURDF("../iota/absolute/iota.urdf",*carpos3)

numJoints = p.getNumJoints(iota1)
for joint in range(numJoints):
    print(p.getJointInfo(iota1, joint))

targetVel = 0.001
maxForce = 0.125
extra = 0

pos1,orie1 = p.getBasePositionAndOrientation(iota1,pClient)

pos2, orie2 = p.getBasePositionAndOrientation(iota2,pClient)

diff12 = [0,0,0]
diff21 = [0,0,0]

for i in range(3):
    diff12[i] = (pos1[i] - pos2[i])/2
    diff21[i] = (pos2[i] - pos1[i])/2
print(diff12,diff21)
cid = p.createConstraint(iota1,
                        -1,                 ## front docking plate
                        iota2,
                        -1,                 ## back docking plate
                        p.JOINT_FIXED,
                        [0,0,0],
                        diff21,
                        diff12)

'''cid2 = p.createConstraint(iota3,
                         25,
                         iota2,
                         11,
                         p.JOINT_FIXED,
                         [1,0,0],
                         [0,0,0],
                         [0.05,0,0])
'''
while(1):

    #p.setJointMotorControl2(iota,1  ,p.VELOCITY_CONTROL, targetVelocity=targetVel, force=maxForce)
    #p.setJointMotorControl2(iota,5  ,p.VELOCITY_CONTROL, targetVelocity=targetVel, force=maxForce)
    #p.setJointMotorControl2(iota,3  ,p.VELOCITY_CONTROL, targetVelocity=-targetVel, force=maxForce)
    #p.setJointMotorControl2(iota,7  ,p.VELOCITY_CONTROL, targetVelocity=-targetVel, force=maxForce)
    #p.setJointMotorControl2(iota,16  ,p.VELOCITY_CONTROL, targetVelocity=10*targetVel, force=maxForce)
    #p.setJointMotorControl2(iota,11  ,p.VELOCITY_CONTROL, targetVelocity=10*targetVel, force=maxForce)
    p.stepSimulation()
    time.sleep(0.05)
