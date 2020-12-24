import cv2
import threading
import time
import pybullet as p
class pybullet_Camera:

    def __init__(self,pos, target_pos, up_vec, width=640, height=360, pClient=None,sleep_rate=0.7,frame_rate=None, sync=True):
        self.camView = p.computeViewMatrix(cameraEyePosition = pos,
                                      cameraTargetPosition = target_pos,
                                      cameraUpVector = up_vec,
                                      physicsClientId = pClient
                                      )
        self.camProj = p.computeProjectionMatrixFOV(fov = 90,
                                               aspect = 1.778,
                                               nearVal = 0.1,
                                               farVal = 6,
                                               physicsClientId = pClient
                                               )
        self.pClient = pClient or 0
        self.WIDTH, self.HEIGHT = width, height
        self.lock = threading.Lock()
        self.running = False
        self.grabbed = False
        self.sleep_rate = sleep_rate
        if frame_rate is not None:
            print('Warning: you should not set frame rate manually for hardware :)')
            self.frame_rate = frame_rate
        else:
            self.frame_rate = 30
        self.self_info = [pos, target_pos, up_vec]
        self.sync = sync

    def __refresh(self,flag):
        while self.running:
            self.lock.acquire()
            try:
                self.prev = self.get_image()
                if not self.grabbed: self.grabbed = True
            finally:
                self.lock.release()
            time.sleep(self.sleep_rate/self.frame_rate) ## This is very neccesary so that the thread doesnt consume the whole cpu time
        return True

    def read(self):
        if not self.sync:
            return self.get_image()
        self.lock.acquire()
        try:
            if not self.grabbed:
                self.grabbed=True
                self.prev = self.get_image()
        finally:
            self.lock.release()
        return self.prev

    def connect(self):
        self.running = True
        if self.sync:
            refresh_loop = threading.Thread(target= self.__refresh, args=(1,))
            refresh_loop.start()

    def close(self):
        self.running = False
    def get_image(self):
        return p.getCameraImage(      width = self.WIDTH,
                                      height = self.HEIGHT,
                                      viewMatrix = self.camView,
                                      projectionMatrix = self.camProj,
                                      renderer = p.ER_BULLET_HARDWARE_OPENGL,
                                      physicsClientId = self.pClient
                                      ) [2]


    def setter(self,propid, value):
        '''
        This is for hardware to control camera apis
        Dont use it in simulation
        '''
        retval = False
        if self.running:
            print("The camera is being used in the loop can't change the settings\n Try closing the connection and trying again!")
        else:
            self.lock.acquire()
            try:
                retval = self.cap.set(propid,value)
                if not retval:
                    print("failed to set the following attribute to "+str(value)+" !")
            finally:
                self.lock.release()
        return retval

    def __del__(self):
        self.close()
