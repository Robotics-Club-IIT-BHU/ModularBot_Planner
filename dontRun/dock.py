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
    max_vel = 10
    min_vel = 1
    motor_force = 10
    dock_encoders = [0,0]
    servo_force = 10
    def __init__(self, path=None,init_pos = None, physicsClient=None):
        if path is None:
            path="../iota/absolute/iota.urdf"
        self.pClient = physicsClient
        self.vel = [12,1e-9]
        #self.vel = [6*(rnd()-0.5)/0.5, 6*(rnd()-0.5)/0.5]
        theta = np.arctan(self.vel[1]/self.vel[0]) +np.pi
        if self.vel[0] < 0 and self.vel[1]>0:
            theta += np.pi
        orie = p.getQuaternionFromEuler((0,0,theta))
        if init_pos is None:
            init_pos = self.init_pos()
        self.id = p.loadURDF(path,
                            basePosition=init_pos,
                            #baseOrientation=orie,
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
                                        targetVelocity=self.signum_fn(for_vec[i]+rot_vec[i]),
                                        force=100,
                                        physicsClientId=self.pClient)
        self.vel = vel_vec
        #p.setJointMotorControl2()
    def signum_fn(self,val):
        if val>0:
            if val > self.min_vel:
                return min(self.max_vel,val)
            else:
                return 0
        else:
            if val < -1*self.min_vel:
                return max(-1*self.max_vel, val)
            else:
                return 0
    def dock(self,other):
        self.dockees.append(other)
        other.dockees.append(self)
        pos11,_ = p.getBasePositionAndOrientation(self.id,self.pClient)

        pos22, _ = p.getBasePositionAndOrientation(other.id,other.pClient)
        diff12 = [0,0,0]
        diff21 = [0,0,0]

        for i in range(3):
            diff12[i] = (pos11[i] - pos22[i])/2
            diff21[i] = (pos22[i] - pos11[i])/2

        cid = p.createConstraint(self.id,
                                -1,                 ## front docking plate
                                other.id,
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
            raise Exception(str(self.id)+" "+str(other.id )+", These two modules have not been docked before")
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

    def dock_servo(self,dock_id,angle):
        self.dock_encoders[dock_id] = angle
        while True:
            curr = p.getJointState(self.id,
                                  self.docks[dock_id],
                                  self.pClient)[0]
            vel = 10*(angle - curr)
            if vel < 0.02:
                self.dock_encoders[dock_id] = curr
                break
            p.setJointMotorControl2(bodyUniqueId=self.id,
                                    jointIndex=self.docks[dock_id],
                                    controlMode=p.VELOCITY_CONTROL,
                                    targetVelocity=vel,
                                    force=self.servo_force,
                                    physicsClientId=self.pClient)
            p.stepSimulation(self.pClient)
            time.sleep(0.05)
    def stop(self):
        for i,wheel_set in enumerate([self.r_wheels, self.l_wheels]):
            for wheel in wheel_set:
                p.setJointMotorControl2(self.id,
                                        wheel,
                                        controlMode=p.VELOCITY_CONTROL,
                                        targetVelocity=0,
                                        force=self.motor_force,
                                        physicsClientId=self.pClient)
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
iota1 = iOTA("../iota/absolute/iota.urdf",init_pos = carpos, physicsClient=pClient)
iota2 = iOTA("../iota/absolute/iota.urdf",init_pos = carpos2, physicsClient=pClient)
#iota3 = p.loadURDF("../iota/absolute/iota.urdf",*carpos3)

numJoints = p.getNumJoints(iota1.id)
for joint in range(numJoints):
    print(p.getJointInfo(iota1.id, joint))

targetVel = 0.001
maxForce = 0.125
extra = 0

pos1,orie1 = p.getBasePositionAndOrientation(iota1.id,pClient)

pos2, orie2 = p.getBasePositionAndOrientation(iota2.id,pClient)
euler2 = p.getEulerFromQuaternion(orie2, pClient)
r = 0.01
print("euler",euler2)
setpoint = [pos2[0] - r*np.cos(euler2[1])*np.cos(euler2[2]), pos2[1] - r*np.cos(euler2[1])*np.sin(euler2[2]), pos2[2] - r*np.sin(euler2[1])]
print("setpoint",setpoint)
#print(diff12,diff21)
'''cid = p.createConstraint(iota1,
                        -1,                 ## front docking plate
                        iota2,
                        -1,                 ## back docking plate
                        p.JOINT_FIXED,
                        [0,0,0],
                        diff21,
                        diff12)

cid2 = p.createConstraint(iota3,
                         25,
                         iota2,
                         11,
                         p.JOINT_FIXED,
                         [1,0,0],
                         [0,0,0],
                         [0.05,0,0])
'''


while True:
    #p.setJointMotorControl2(iota,1  ,p.VELOCITY_CONTROL, targetVelocity=targetVel, force=maxForce)
    #p.setJointMotorControl2(iota,5  ,p.VELOCITY_CONTROL, targetVelocity=targetVel, force=maxForce)
    #p.setJointMotorControl2(iota,3  ,p.VELOCITY_CONTROL, targetVelocity=-targetVel, force=maxForce)
    #p.setJointMotorControl2(iota,7  ,p.VELOCITY_CONTROL, targetVelocity=-targetVel, force=maxForce)
    #p.setJointMotorControl2(iota,16  ,p.VELOCITY_CONTROL, targetVelocity=10*targetVel, force=maxForce)
    #p.setJointMotorControl2(iota,11  ,p.VELOCITY_CONTROL, targetVelocity=10*targetVel, force=maxForce)
    pos1, orie1 = p.getBasePositionAndOrientation(iota1.id,pClient)
    if distance(pos1,setpoint)<0.005:
        iota1.stop()
        #iota1.dock_servo(1,np.pi)
        break
    vec = [50*(setpoint[i]-pos1[i]) for i in range(3)]
    iota1.control(vec)
    for i in range(10):p.stepSimulation()
    time.sleep(0.05)
r = 0.005
iota1.stop()
iota1.dock(iota2)
iota2.stop()
'''
setpoint = [pos2[0] - r*np.cos(euler2[1])*np.cos(euler2[2]), pos2[1] - r*np.cos(euler2[1])*np.sin(euler2[2]), pos2[2] - r*np.sin(euler2[1])]
print(setpoint)
while True:
    pos1, orie1 = p.getBasePositionAndOrientation(iota1.id,pClient)
    if distance(pos1,setpoint)<0.005:
        break
    vec = [100*(setpoint[i]-pos1[i]) for i in range(3)]
    iota1.control(vec)
    for i in range(10):p.stepSimulation()
    time.sleep(0.05)
print("done")
iota1.stop()
iota1.dock(iota2)
iota2.stop()
'''
while True:
    p.stepSimulation()
