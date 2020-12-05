import pybullet as p
import pybullet_data
import time

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadURDF("plane.urdf")
p.setGravity(0,0,-10)
carpos = [0,0,0.1]
iota = p.loadURDF("iota.urdf",*carpos)
numJoints = p.getNumJoints(iota)
for joint in range(numJoints):
    print(p.getJointInfo(iota, joint))

targetVel = 50
maxForce = 1000
extra = 0
while(1):

    p.setJointMotorControl2(iota,15  ,p.VELOCITY_CONTROL, targetVelocity=targetVel, force=maxForce)
    p.setJointMotorControl2(iota,19  ,p.VELOCITY_CONTROL, targetVelocity=targetVel, force=maxForce)
    p.setJointMotorControl2(iota,17  ,p.VELOCITY_CONTROL, targetVelocity=-targetVel, force=maxForce)
    p.setJointMotorControl2(iota,21  ,p.VELOCITY_CONTROL, targetVelocity=-targetVel, force=maxForce)
    p.setJointMotorControl2(iota,25  ,p.VELOCITY_CONTROL, targetVelocity=10*targetVel, force=maxForce) 
    p.setJointMotorControl2(iota,11  ,p.VELOCITY_CONTROL, targetVelocity=10*targetVel, force=maxForce)
    p.stepSimulation()
    time.sleep(0.05)
