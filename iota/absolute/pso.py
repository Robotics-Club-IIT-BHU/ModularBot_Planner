import pybullet as p
import pybullet_data
from math import sqrt
import numpy as np
rnd = np.random.random

class iOTA():
    arena_x = 5
    arena_y = 5
    l_wheels = [15, 19]
    r_wheels = [17, 21]
    docks = [25, 11]
    def __init__(self, path=None, physicsClient=None):
        if path is None:
            path="urdf/iota.urdf"
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

def distance(pos, target):
    return ((pos[0]-target[0])**2+(pos[1]-target[1])**2)

pClient = p.connect(p.GUI)


p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-10)
p.loadURDF('plane.urdf')
iotas = [ iOTA("iota.urdf", physicsClient=pClient) for i in range(40) ]
box = p.loadURDF("dabba.urdf", basePosition=((5*(rnd()-0.5)/0.5),(5*(rnd()-0.5)/0.5),0.08 ))
target = p.getBasePositionAndOrientation(box, pClient)[0]
#poses = [list(p.getBasePositionAndOrientation(iota.id, pClient)[0]) for iota in iotas]
particle_optim = [ list(p.getBasePositionAndOrientation(iota.id)[0]) for iota in iotas ]
global_optim = (0,0,0.08)
best = 1000000000

w, tp , tg = 0.999, 1.2, 1.5
for opt, iota in zip(particle_optim, iotas):
    if iota.dist(target)<best:
        global_optim = opt
        best = iota.dist(target)
k = 0
while True:
    for i, (opt, iota) in enumerate(zip(particle_optim, iotas)):
        vel_vec = [0, 0]
        for j in range(2):
            rp, rg = rnd(),rnd()
            loc = p.getBasePositionAndOrientation(iota.id, physicsClientId=pClient)[0][j]
            vel_vec[j] = w*iota.vel[j] + tp*rp*(opt[j] - loc) + tg*rg*(global_optim[j] - loc)
        iota.control(vel_vec)
    for i in range(10):
        p.stepSimulation()
    for i, (opt, iota) in enumerate(zip(particle_optim, iotas)):
        dst = iota.dist(target)
        if dst < distance(opt,target):
            particle_optim[i] = p.getBasePositionAndOrientation(iota.id, physicsClientId=pClient)[0]
        if dst < best:
            global_optim = p.getBasePositionAndOrientation(iota.id, physicsClientId=pClient)[0]
            best = dst
            print(best)
    if best < 0.14999:
        break
    if k%1000==0:
        print(best)
    k+=1
print(target,'\n --- \n' ,global_optim)
