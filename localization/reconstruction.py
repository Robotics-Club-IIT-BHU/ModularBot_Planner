import cv2
import numpy as np
from vector import Vect2d, Vect3d

def get_pos(mask, camera_coor=None, camera_Rot=None, cameraMatrix=None,draw=False,img=None):
    '''
    This calculates the exact position of the masked object
    '''

    if camera_coor is None and camera_Rot is None:
        raise Exception('Either Camera\'s coordinates or Camera\'s Rotation Matrix has to be passed')
    if draw and img is None:
        raise Exception('For the the contour to be drawn pass the img parameter also.')
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(9,5))
    dilated = cv2.dilate(mask,kernel,iterations=5)
    cnts, heirs = cv2.findContours(dilated, cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    mc = []
    for cnt in cnts:
        M = cv2.moments(cnt)
        try:
            m = [ M['m10']/(M['m00']), M['m01']/(M['m00']) ]
        except:
            m = None
        if m is not None:
            mc.append(m)

    mc = np.array(mc).reshape(2,-1).mean(axis=-1)
    if draw:
        cv2.circle(img, (int(mc[0]),int(mc[1])), 5, (255,0,0) , 2 )
    mc = np.array([ *mc, 1.0]).reshape(3,1)
    default_z = 0.04
    s = 0.01
    P = np.linalg.inv(cameraMatrix)@(s*mc)
    P -= camera_Rot[1]
    P = np.linalg.inv(camera_Rot[0])@(P)
    z = P[-1,0]
    while abs(z-default_z)>0.1*default_z:
        P = np.linalg.inv(cameraMatrix)@(s*mc)
        P -= camera_Rot[1]
        P = np.linalg.inv(camera_Rot[0])@(P)
        z = P[-1,0]
    return P

def vec2rotm(point_vec, up_vec):
    if not isinstance(up_vec, Vect3d):
        up_vec = Vect3d(*up_vec)
    if not isinstance(point_vec, Vect3d):
        point_vec = Vect3d(*point_vec)
    r_y = up_vec*point_vec
    print("up_vec", up_vec, "point_vec", point_vec, "r_y", r_y)
    return np.array([
                    [ *point_vec.my_unit().get_values()],
                    [ *r_y.my_unit().get_values()],
                    [ *up_vec.my_unit().get_values()],
                    ])

def euler2rotm(theta):
    R_x = np.array([[1,         0,                  0                   ],
                    [0,         math.cos(theta[0]), -math.sin(theta[0]) ],
                    [0,         math.sin(theta[0]), math.cos(theta[0])  ]
                    ])
    R_y = np.array([[math.cos(theta[1]),    0,      math.sin(theta[1])  ],
                    [0,                     1,      0                   ],
                    [-math.sin(theta[1]),   0,      math.cos(theta[1])  ]
                    ])
    R_z = np.array([[math.cos(theta[2]),    -math.sin(theta[2]),    0],
                    [math.sin(theta[2]),    math.cos(theta[2]),     0],
                    [0,                     0,                      1]
                    ])
    R = np.dot(R_z, np.dot( R_y, R_x ))

    return R
