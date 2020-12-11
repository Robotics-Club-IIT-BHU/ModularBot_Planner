#!/usr/bin/python
# -*- coding: utf-8 -*-
import numpy as np
import pybullet as p
import time
import math
import pybullet_data
physicsClient = p.connect(p.GUI)

p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -10)


def quaternion_to_euler(
    x,
    y,
    z,
    w,
    ):

    import math
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    X = math.degrees(math.atan2(t0, t1))

    t2 = +2.0 * (w * y - z * x)
    t2 = (+1.0 if t2 > +1.0 else t2)
    t2 = (-1.0 if t2 < -1.0 else t2)
    Y = math.degrees(math.asin(t2))

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    Z = math.degrees(math.atan2(t3, t4))

    return (X, Y, Z)


# def vel_pos (body1,body2):
 #    position1, orientation1 = p.getBasePositionAndOrientation(body1)
  #   position, orientation = p.getBasePositionAndOrientation(body2)
   #  euler= quaternion_to_euler(orientation[0], orientation[1], orientation[2], orientation[3])
    # diff = math.atan2((position1[1]-position[1]),(position1[0]-position[0]))
    # diff_dash = diff
    # error = math.sqrt(((position1[1]-position[1])**2) +((position1[0]-position[0])**2))
 #    ome = diff-(euler[2]*3.14/180)
  #   ome1 = math.atan2(math.sin(ome), math.cos(ome))
 #    omega = 0.5*ome1
   #  vel = 0.5
    # right_v = (2*vel + omega*0.3)/0.2
     # left_v =(2*vel - omega*0.3)/0.2

    # return right_v,left_v,error

def vel_pos(position1, position, orientation):

    euler = quaternion_to_euler(orientation[0], orientation[1],
                                orientation[2], orientation[3])
    diff = math.atan2(position1[1] - position[1], position1[0]
                      - position[0])
    diff_dash = diff
    error = math.sqrt((position1[1] - position[1]) ** 2 + (position1[0]
                      - position[0]) ** 2)
    ome = diff - euler[2] * 3.14 / 180
    ome1 = math.atan2(math.sin(ome), math.cos(ome))
    omega = 0.5 * ome1
    vel = 0.8
    right_v = (2 * vel + omega * 0.4) / 0.2
    left_v = (2 * vel - omega * 0.4) / 0.2
    print(right_v, left_v)
    return (right_v, left_v, error)


def loc(body1, body2):
    (position1, orientation1) = p.getBasePositionAndOrientation(body1)
    (position, orientation) = p.getBasePositionAndOrientation(body2)
    (right, left, error) = vel_pos(position1, position, orientation)
    return (right, left, error)


planeID = p.loadURDF('plane.urdf')
dabba = p.loadURDF('dabba.urdf', basePosition=[8, 6, 0])
car = p.loadURDF('car2.urdf')
car2 = p.loadURDF('car2.urdf', basePosition=[1, 3, 0])
car3 = p.loadURDF('car2.urdf', basePosition=[2, 4, 0])
number_of_joints = p.getNumJoints(car)
for joint_number in range(number_of_joints):
    info = p.getJointInfo(car, joint_number)
    print(info)
(position, orientation) = p.getBasePositionAndOrientation(car)
print(orientation)

error = 1

wheel_indices = [4, 5]

while True:
    while 1:
        keys = p.getKeyboardEvents()
        for (k, v) in keys.items():
            if k == ord('p') and v & p.KEY_IS_DOWN:  #
                position1 = [0, 0, 0]
                (position, orientation) = \
                    p.getBasePositionAndOrientation(car)
                error = math.sqrt((position1[1] - position[1]) ** 2
                                  + (position1[0] - position[0]) ** 2)

                if error > 0.3:
                    (right_vel, left_vel, error) = vel_pos(position1,
                            position, orientation)  # velocities to make car follow dabba

                    p.setJointMotorControl2(car, 4, p.VELOCITY_CONTROL,
                            targetVelocity=left_vel)

                    p.setJointMotorControl2(car, 5, p.VELOCITY_CONTROL,
                            targetVelocity=right_vel)
                    p.stepSimulation()
                else:

                    for joint in range(4, 6):
                        p.setJointMotorControl2(car, joint,
                                p.VELOCITY_CONTROL, targetVelocity=0)
                    p.stepSimulation()

                position1 = [0, 2, 0]
                (position, orientation) = \
                    p.getBasePositionAndOrientation(car2)
                error = math.sqrt((position1[1] - position[1]) ** 2
                                  + (position1[0] - position[0]) ** 2)  # velocities to make car2 follow car1

                if error > 0.5:
                    (right_vel, left_vel, error) = vel_pos(position1,
                            position, orientation)  # velocities to make car follow dabba

                    p.setJointMotorControl2(car2, 4,
                            p.VELOCITY_CONTROL, targetVelocity=left_vel)

                    p.setJointMotorControl2(car2, 5,
                            p.VELOCITY_CONTROL,
                            targetVelocity=right_vel)
                    p.stepSimulation()
                else:

                    for joint in range(4, 6):
                        p.setJointMotorControl2(car2, joint,
                                p.VELOCITY_CONTROL, targetVelocity=0)
                    p.stepSimulation()

                position1 = [0, 1, 0]
                (position, orientation) = \
                    p.getBasePositionAndOrientation(car3)
                error = math.sqrt((position1[1] - position[1]) ** 2
                                  + (position1[0] - position[0]) ** 2)  # velocities to make car2 follow car1

                if error > 0.5:
                    (right_vel, left_vel, error) = vel_pos(position1,
                            position, orientation)  # velocities to make car follow dabba

                    p.setJointMotorControl2(car3, 4,
                            p.VELOCITY_CONTROL, targetVelocity=left_vel)

                    p.setJointMotorControl2(car3, 5,
                            p.VELOCITY_CONTROL,
                            targetVelocity=right_vel)
                    p.stepSimulation()
                else:

                    for joint in range(4, 6):
                        p.setJointMotorControl2(car3, joint,
                                p.VELOCITY_CONTROL, targetVelocity=0)
                    p.stepSimulation()

            if k == ord('c') and v & p.KEY_IS_DOWN:  #

                (pos1, or1) = p.getBasePositionAndOrientation(car)
                (pos2, or2) = p.getBasePositionAndOrientation(car2)
                (pos3, or3) = p.getBasePositionAndOrientation(car3)
                print(pos1, pos2, pos3)
                position_c = [0, 0, 0]
                position_c[0] = (pos1[0] + pos2[0] + pos3[0]) / 3
                position_c[1] = (pos1[1] + pos2[1] + pos3[1]) / 3
                position_c[2] = (pos1[2] + pos2[2] + pos3[2]) / 3

                print('position_c', position_c)
                (position1, orientation1) = \
                    p.getBasePositionAndOrientation(car)
                error = math.sqrt((position_c[1] - position1[1]) ** 2
                                  + (position_c[0] - position1[0]) ** 2)

                if error > 0.5:
                    (right_vel, left_vel, error) = vel_pos(position_c,
                            position1, orientation1)  # velocities to make car follow dabba

                    p.setJointMotorControl2(car, 4, p.VELOCITY_CONTROL,
                            targetVelocity=left_vel)

                    p.setJointMotorControl2(car, 5, p.VELOCITY_CONTROL,
                            targetVelocity=right_vel)
                    p.stepSimulation()
                else:

                    for joint in range(4, 6):
                        p.setJointMotorControl2(car, joint,
                                p.VELOCITY_CONTROL, targetVelocity=0)
                    p.stepSimulation()

                (position2, orientation2) = \
                    p.getBasePositionAndOrientation(car2)
                error2 = math.sqrt((position_c[1] - position2[1]) ** 2
                                   + (position_c[0] - position2[0])
                                   ** 2)  # velocities to make car2 follow car1

                if error2 > 0.5:
                    (right_vel, left_vel, error2) = vel_pos(position_c,
                            position2, orientation2)  # velocities to make car follow dabba

                    p.setJointMotorControl2(car2, 4,
                            p.VELOCITY_CONTROL, targetVelocity=left_vel)

                    p.setJointMotorControl2(car2, 5,
                            p.VELOCITY_CONTROL,
                            targetVelocity=right_vel)
                    p.stepSimulation()
                else:

                    for joint in range(4, 6):
                        p.setJointMotorControl2(car2, joint,
                                p.VELOCITY_CONTROL, targetVelocity=0)
                    p.stepSimulation()

                (position3, orientation3) = \
                    p.getBasePositionAndOrientation(car3)
                error3 = math.sqrt((position_c[1] - position3[1]) ** 2
                                   + (position_c[0] - position3[0])
                                   ** 2)  # velocities to make car2 follow car1

                if error3 > 1:
                    (right_vel, left_vel, error3) = vel_pos(position_c,
                            position3, orientation3)  # velocities to make car follow dabba

                    p.setJointMotorControl2(car3, 4,
                            p.VELOCITY_CONTROL, targetVelocity=left_vel)

                    p.setJointMotorControl2(car3, 5,
                            p.VELOCITY_CONTROL,
                            targetVelocity=right_vel)
                    p.stepSimulation()
                else:

                    for joint in range(4, 6):
                        p.setJointMotorControl2(car3, joint,
                                p.VELOCITY_CONTROL, targetVelocity=0)
                    p.stepSimulation()

         #       p.setJointMotorControl2(car, joint, p.VELOCITY_CONTROL, targetVelocity =targetVel,force = maxForce)

          #  p.stepSimulation()
        # if (k == p.B3G_UP_ARROW and (v & p.KEY_WAS_RELEASED)):
         #   targetVel = 0
          #  for joint in range(2, 6):
           #     p.setJointMotorControl2(car, joint, p.VELOCITY_CONTROL,targetVelocity = targetVel,force = maxForce)

           # p.stepSimulation()

            if k == p.B3G_DOWN_ARROW:  # and (v & p.KEY_IS_DOWN)
                (position1, orientation1) = \
                    p.getBasePositionAndOrientation(dabba)
                (position, orientation) = \
                    p.getBasePositionAndOrientation(car)
                error = math.sqrt((position1[1] - position[1]) ** 2
                                  + (position1[0] - position[0]) ** 2)

                if error > 1:
                    (right_vel, left_vel, error) = loc(dabba, car)  # velocities to make car follow dabba

                    p.setJointMotorControl2(car, 4, p.VELOCITY_CONTROL,
                            targetVelocity=left_vel)

                    p.setJointMotorControl2(car, 5, p.VELOCITY_CONTROL,
                            targetVelocity=right_vel)

       # velocities to make car2 follow car1

                    (right_v2, left_v2, error_2) = loc(car, car2)

                    p.setJointMotorControl2(car2, 4,
                            p.VELOCITY_CONTROL, targetVelocity=left_v2)

                    p.setJointMotorControl2(car2, 5,
                            p.VELOCITY_CONTROL, targetVelocity=right_v2)

                    (right_v3, left_v3, error_3) = loc(car2, car3)

                    p.setJointMotorControl2(car3, 4,
                            p.VELOCITY_CONTROL, targetVelocity=left_v3)

                    p.setJointMotorControl2(car3, 5,
                            p.VELOCITY_CONTROL, targetVelocity=right_v3)
                    p.stepSimulation()
                else:

                    for joint in range(4, 6):
                        p.setJointMotorControl2(car, joint,
                                p.VELOCITY_CONTROL, targetVelocity=0)

                    if error_2 > 1:

                        (right_v2, left_v2, error_2) = loc(car, car2)

                        p.setJointMotorControl2(car2, 4,
                                p.VELOCITY_CONTROL,
                                targetVelocity=left_v2)

                        p.setJointMotorControl2(car2, 5,
                                p.VELOCITY_CONTROL,
                                targetVelocity=right_v2)

                        (right_v3, left_v3, error_3) = loc(car2, car3)

                        p.setJointMotorControl2(car3, 4,
                                p.VELOCITY_CONTROL,
                                targetVelocity=left_v3)

                        p.setJointMotorControl2(car3, 5,
                                p.VELOCITY_CONTROL,
                                targetVelocity=right_v3)

                        p.stepSimulation()
                    else:

                        for joint in range(4, 6):
                            p.setJointMotorControl2(car2, joint,
                                    p.VELOCITY_CONTROL,
                                    targetVelocity=0)

                        if error_3 > 1:

                            (right_v3, left_v3, error_3) = loc(car2,
                                    car3)

                            p.setJointMotorControl2(car3, 4,
                                    p.VELOCITY_CONTROL,
                                    targetVelocity=left_v3)

                            p.setJointMotorControl2(car3, 5,
                                    p.VELOCITY_CONTROL,
                                    targetVelocity=right_v3)

                            p.stepSimulation()
                        else:

                            for joint in range(4, 6):
                                p.setJointMotorControl2(car3, joint,
                                        p.VELOCITY_CONTROL,
                                        targetVelocity=0)

        p.stepSimulation()
