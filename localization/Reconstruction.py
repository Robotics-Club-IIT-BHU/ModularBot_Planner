import cv2
import numpy as np
from vector import Vect2d, Vect3d
def get_pos(arr_masks, camera_coor=None, camera_Rot=None, cameraMatrix=None):
    if camera_coor is None and camera_Rot is None:
        raise Exception('Either Camera\'s coordinates or Camera\'s Rotation Matrix has to be passed')
    '''
    This calculates the exact position of the masked object
    '''
    return [0,0]

def vec2rotm(point_vec, up_vec):
    if not isinstance(up_vec, Vect3d):
        up_vec = Vect3d(*up_vec)
    if not isinstance(point_vec, Vect3d):
        point_vec = Vect3d(*point_vec)
    r_y = up_vec*point_vec
    return np.array([
                    [ *point_vec.get_values()],
                    [ *r_y.get_values()],
                    [ *up_vec.get_values()],
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
