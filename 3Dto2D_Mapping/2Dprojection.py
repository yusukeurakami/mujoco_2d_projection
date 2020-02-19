import numpy as np
import matplotlib.pyplot as plt

def global2label(obj_pos, cam_pos, cam_ori, fov=60):
    """
    :param obj_pos: 3D coordinates of the joint in nparray [m]
    :param cam_pos: 3D coordinates of the camera in nparray [m]
    :param cam_ori: camera 3D rotation (Rotation order of x->y->z) in nparray [rad]
    :param fov: field of view in integer [degree]
    :return: Heatmap of the object in the 2D pixel space.
    """
    e = np.array([128,128,1])
    fov = np.array([fov])

    # Converting the MuJoCo coordinate into typical computer vision coordinate.
    cam_ori_cv = np.array([cam_ori[1], cam_ori[0], -cam_ori[2]])
    obj_pos_cv = np.array([obj_pos[1], obj_pos[0], -obj_pos[2]])
    cam_pos_cv = np.array([cam_pos[1], cam_pos[0], -cam_pos[2]])

    obj_pos_in_2D, obj_pos_from_cam = get_2D_from_3D(obj_pos_cv, cam_pos_cv, cam_ori_cv, fov, e)
    label = gkern(e[0]/2, e[1]/2, (obj_pos_in_2D[1]/4, (e[0]*2-obj_pos_in_2D[0])/4), s=1)
    return label

def get_2D_from_3D(a, c, theta, fov, e):
    """
    :param a: 3D coordinates of the joint in nparray [m]
    :param c: 3D coordinates of the camera in nparray [m]
    :param theta: camera 3D rotation (Rotation order of x->y->z) in nparray [rad]
    :param fov: field of view in integer [degree]
    :param e: 
    :return:
        - (bx, by) ==> 2D coordinates of the obj [pixel]
        - d ==> 3D coordinates of the joint (relative to the camera) [m]
    """

    # Get the vector from camera to object in global coordinate.
    ac_diff = a - c

    # Rotate the vector in to camera coordinate
    x_rot = np.array([[1 ,0, 0],
                    [0, np.cos(theta[0]), np.sin(theta[0])],
                    [0, -np.sin(theta[0]), np.cos(theta[0])]])

    y_rot = np.array([[np.cos(theta[1]) ,0, -np.sin(theta[1])],
                [0, 1, 0],
                [np.sin(theta[1]), 0, np.cos(theta[1])]])

    z_rot = np.array([[np.cos(theta[2]) ,np.sin(theta[2]), 0],
                [-np.sin(theta[2]), np.cos(theta[2]), 0],
                [0, 0, 1]])

    transform = z_rot.dot(y_rot.dot(x_rot))
    d = transform.dot(ac_diff)    

    # scaling of projection plane using fov
    fov_rad = np.deg2rad(fov)    
    e[2] *= e[1]*1/np.tan(fov_rad/2.0)

    # Projection from d to 2D
    bx = e[2]*d[0]/(d[2]) + e[0]
    by = e[2]*d[1]/(d[2]) + e[1]

    return (bx, by), d

def gkern(h, w, center, s=4):
    x = np.arange(0, w, 1, float)
    y = np.arange(0, h, 1, float)
    y = y[:, np.newaxis]
    x0 = center[0]
    y0 = center[1]
    return np.exp(-1 * ((x - x0) ** 2 + (y - y0) ** 2) / s**2)

if __name__ == "__main__":
    obj_pos = np.array([0.2, 0.25, 1.03])
    cam_pos = np.array([0.7, 0, 1.5])
    cam_ori = np.array([0,1.1,1.57])
    fov = 60
    
    label = global2label(obj_pos, cam_pos, cam_ori, fov)
    plt.imshow(label)
    plt.show()
