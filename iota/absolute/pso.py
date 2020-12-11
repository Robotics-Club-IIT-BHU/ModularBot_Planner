import pybullet as p
import pybullet_data
import numpy as np
rnd = np.random.random

class iOTA():
    arena_x = 3
    arena_y = 3
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
        self.id = p.loadURDF(path, basePosition=self.init_pos(), baseOrientation=orie , self.pClient)

    def init_pos(self):
        return [arena_x*(rnd()-0.5)/0.5,
                arena_y*(rnd()-0.5)/0.5,
                0.001
                ]
    def dist(self, target):
        pos = p.getBasePositionAndOrientation(self.id, self.pClient)[0]
        return sqrt((target[0]-pos[0])**2 + (target[1]-pos[1])**2)
    def control(self,velocity_vector):
        '''
        Implement differencial drive for one time step
        '''
        pass


pClient = p.connect(p.GUI)


p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-10)
iotas = [ iOTA("iota.urdf") for i in range(6) ]
box = p.loadURDF("dabba.urdf", )
particle_optim = [ list(p.getBasePositionAndOrientation(iota.id)[0]) for iota in iotas]
global_optim = [0,0,0.08]
best = 1000000000
for opt, iota in zip(particle_optim, iotas):
    if iota.dist(target)<best:
        global_optim = opt
        best = iota.dist(target)
while True:


    p.stepSimulation()
