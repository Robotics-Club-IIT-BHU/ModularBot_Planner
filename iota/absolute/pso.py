import pybullet as p
import pybullet_data
import numpy as np
import math
rnd = np.random.random

class iOTA():
    arena_x = 2
    arena_y = 2
    l_wheels = [15, 19]
    r_wheels = [17, 21]
    docks = [25, 11]
    def __init__(self, path=None, physicsClient=None):
        if path is None:
            path="urdf/iota.urdf"
        self.pClient = physicsClient
        self.vel = [(rnd()-0.5)/0.5, (rnd()-0.5)/0.5]
        theta = np.arctan(self.vel[1]/self.vel[0])
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
        return math.sqrt((target[0]-pos[0])**2 + (target[1]-pos[1])**2)
    def control(self,velocity_vector):
        '''
        Implement differencial drive for one time step
        '''
        pass

def distance(pos, target):
    return ((pos[0]-target[0])**2+(pos[1]-target[1])**2)

pClient = p.connect(p.GUI)


p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadURDF('plane.urdf')
p.setGravity(0,0,-10)
iotas = [ iOTA("iota.urdf",physicsClient=pClient) for i in range(6) ]
box = p.loadURDF("dabba.urdf", basePosition=(1,0,0.1))
target = p.getBasePositionAndOrientation(box, pClient)[0]
particle_optim = [ list(p.getBasePositionAndOrientation(iota.id)[0]) for iota in iotas]
global_optim = (0,0,0.08)
best = 1000000000

w, tp , tg = 0.4, 0.3, 0.3
for opt, iota in zip(particle_optim, iotas):
    if iota.dist(target)<best:
        global_optim = opt
        best = iota.dist(target)
while True:
    for i, (opt, iota) in enumerate(zip(particle_optim, iotas)):
        vel_vec = [0, 0]
        for j in range(2):
            rp, rg = rnd(),rnd()
            loc = p.getBasePositionAndOrientation(iota.id, physicsClientId=pClient)[0][j]
            vel_vec[j] = w*iota.vel[j] + tp*rp*(opt-loc) + tg*rg*(global_optim - loc)
        iota.control(vel_vec)
    p.stepSimulation()
    for i, (opt, iota) in enumerate(zip(particle_optim, iotas)):
        dst = iota.dist(target)
        if dst < distance(opt,target):
            particle_optim[i] = p.getBasePositionAndOrientation(iota.id, physicsClientId=pClient)[0]
            if dst < best:
                global_optim = particle_optim[i]
