'''
We shall read the kitti style annotation files
Draw the annotations
'''

# !/usr/bin/env python
import sys
'''
visualize autonomoose labels in kitti format
note that we don't have a reference camera
all the camera coordinates in the respective camera coordinates. e.g camera 7 has labels in camera 7 coordinates
'''


sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')

import numpy as np
import cv2
import load_calibration
from scipy.spatial.transform import Rotation as R


'''
#Values    Name      Description
----------------------------------------------------------------------------
   1    type         Describes the type of object: 'Car', 'Van', 'Truck',
                     'Pedestrian', 'Person_sitting', 'Cyclist', 'Tram',
                     'Misc' or 'DontCare'
   1    truncated    Float from 0 (non-truncated) to 1 (truncated), where
                     truncated refers to the object leaving image boundaries
   1    occluded     Integer (0,1,2,3) indicating occlusion state:
                     0 = fully visible, 1 = partly occluded
                     2 = largely occluded, 3 = unknown
   1    alpha        Observation angle of object, ranging [-pi..pi]
   4    bbox         2D bounding box of object in the image (0-based index):
                     contains left, top, right, bottom pixel coordinates
   3    dimensions   3D object dimensions: height, width, length (in meters)
   3    location     3D object location x,y,z in camera coordinates (in meters)
   1    rotation_y   Rotation ry around Y-axis in camera coordinates [-pi..pi]
   1    score        Only for results: Float, indicating confidence in
detection, needed for p/r curves, higher is better.
'''


calib_path = '/media/wavelab/d3cd89ab-7705-4996-94f3-01da25ba8f50/autonomoose/calibmoose'
calib = load_calibration.load_calibration(calib_path)
frame = 4
cam = '0'
img_path = "/media/wavelab/d3cd89ab-7705-4996-94f3-01da25ba8f50/autonomoose/processedmoose/image_0" + cam + "/data/" + format(frame, '010') + ".png"
annotations_file = "/media/wavelab/d3cd89ab-7705-4996-94f3-01da25ba8f50/autonomoose/processedmoose/annotation_00/" + format(frame, '010') + ".txt"

# Load 3d annotations
annotations_data = None

DISTORTED = False
if DISTORTED:
    path_type = 'raw'
else:
    path_type = 'processedmoose'

height=[]
width=[]
length =[]
x_center =[]
y_center = []
z_center =[]
rotation_y =[]

x_min =[]
y_min = []
x_max =[]
y_max =[]

with open(annotations_file) as f:
    for line in f:

        #print(f.readline())
        #print(line)
        l = line.split(" ")
        height.append(float(l[8]))
        width.append(float(l[9]))
        length.append(float(l[10]))

        #print(height)
        x_min.append(float(l[4]))
        y_min.append(float(l[5]))
        x_max.append(float(l[6]))
        y_max.append(float(l[7]))
        x_center.append(float(l[11]))
        y_center.append(float(l[12])-float(l[8])/2)
        z_center.append(float(l[13]))

        rotation_y.append(float(l[14]))
f.close()




# Projection matrix from camera to image frame
T_IMG_CAM = np.eye(4); #identity matrix
T_IMG_CAM[0:3,0:3] = np.array(calib['CAM0' + cam]['camera_matrix']['data']).reshape(-1, 3); #camera to image #this comes from e.g "F.yaml" file

#T_IMG_CAM : 4 x 4 matrix
T_IMG_CAM = T_IMG_CAM[0:3,0:4]; # remove last row, #choose the first 3 rows and get rid of the last row

T_CAM_LIDAR = np.linalg.inv(np.array(calib['extrinsics']['T_LIDAR_CAM0' + cam])); #going from lidar to camera

T_LIDAR_CAM = np.array(calib['extrinsics']['T_LIDAR_CAM0' + cam])

T_IMG_LIDAR = np.matmul(T_IMG_CAM, T_CAM_LIDAR); # go from lidar to image

img = cv2.imread(img_path)
img_h, img_w = img.shape[:2]  # get the image height and width

for i in range(len(height)):


    # Add each cuboid to image
    '''
    #Rotations in 3 dimensions can be represented by a sequece of 3 rotations around a sequence of axes.
    #In theory, any three axes spanning the 3D Euclidean space are enough. In practice the axes of rotation are chosen to be the basis vectors.
    #The three rotations can either be in a global frame of reference (extrinsic) or in a body centred frame of refernce (intrinsic),
    #which is attached to, and moves with, the object under rotation
    '''
    # the coordinates in the tracklet json are lidar coords
    #print(x_center)

    T_CAM_Cuboid = np.eye(4);  # identity matrix
    T_CAM_Cuboid[0:3, 0:3] = R.from_euler('y', rotation_y[i], degrees=False).as_dcm();  # rotate about the y axis
    T_CAM_Cuboid[0][3] = x_center[i];  # center of the tracklet, from cuboid to lidar
    T_CAM_Cuboid[1][3] = y_center[i];
    T_CAM_Cuboid[2][3] = z_center[i];
    radius = 3


    T_LIDAR_Cuboid = np.matmul(T_LIDAR_CAM, T_CAM_Cuboid)
    T_Cuboid_LIDAR = np.linalg.inv(T_LIDAR_Cuboid)




    T_IMG_Cuboid = np.matmul(T_IMG_CAM, T_CAM_Cuboid);  #  cameracuboid to image
    x = int(T_IMG_Cuboid[0][3] / T_IMG_Cuboid[2][3]);  # x value/ z
    y = int(T_IMG_Cuboid[1][3] / T_IMG_Cuboid[2][3]);  # y value/z

    cv2.circle(img, (x,y), radius, [0, 0, 255], thickness=2, lineType=8, shift=0); #plot center of the tracklet



    #Very import to remember that The camera frame has z forward, y down and x right. The rotation y should be around the yaxis
    #this is made in the cuboid frame which has the same axis as camera frame
    #note that in the annotations file, the length, width and height are given in the right handed x-y-z coordinate frame. i,e x---length, y ---width, z ----height
    #in our new cuboid frame, which is the same as the camera frame, the above described right handed x-y-z coordinate frame is now z forward, y down and x right,
    #that means, x --- length, y--- height, z --- width

    front_right_bottom = np.array([[1, 0, 0, length[i] / 2], [0, 1, 0, height[i] / 2], [0, 0, 1, width[i] / 2],
                                   [0, 0, 0, 1]]);  # cuboid in camera frame coordinates
    front_right_top = np.array(
        [[1, 0, 0, length[i] / 2], [0, 1, 0, -height[i] / 2], [0, 0, 1, width[i] / 2], [0, 0, 0, 1]]);
    front_left_bottom = np.array(
        [[1, 0, 0, -length[i] / 2], [0, 1, 0, height[i] / 2], [0, 0, 1, width[i] / 2], [0, 0, 0, 1]]);
    front_left_top = np.array(
        [[1, 0, 0, -length[i] / 2], [0, 1, 0, -height[i] / 2], [0, 0, 1, width[i] / 2], [0, 0, 0, 1]]);

    back_right_bottom = np.array(
        [[1, 0, 0, length[i] / 2], [0, 1, 0, height[i] / 2], [0, 0, 1, -width[i] / 2], [0, 0, 0, 1]]);
    back_right_top = np.array(
        [[1, 0, 0, length[i] / 2], [0, 1, 0, -height[i] / 2], [0, 0, 1, -width[i] / 2], [0, 0, 0, 1]]);
    back_left_bottom = np.array(
        [[1, 0, 0, -length[i] / 2], [0, 1, 0, height[i] / 2], [0, 0, 1, -width[i] / 2], [0, 0, 0, 1]]);
    back_left_top = np.array(
        [[1, 0, 0, -length[i] / 2], [0, 1, 0, -height[i] / 2], [0, 0, 1, -width[i] / 2], [0, 0, 0, 1]]);


    #Project to image
    f_r_b  = np.matmul(T_IMG_CAM, np.matmul(T_CAM_Cuboid, front_right_bottom));  # cuboid to lidar, lidar to camera


    f_r_t= np.matmul(T_IMG_CAM, np.matmul(T_CAM_Cuboid, front_right_top));


    f_l_b = np.matmul(T_IMG_CAM, np.matmul(T_CAM_Cuboid, front_left_bottom));



    f_l_t = np.matmul(T_IMG_CAM, np.matmul(T_CAM_Cuboid, front_left_top));



    b_r_b = np.matmul(T_IMG_CAM, np.matmul(T_CAM_Cuboid, back_right_bottom));


    b_r_t = np.matmul(T_IMG_CAM, np.matmul(T_CAM_Cuboid, back_right_top));



    b_l_b = np.matmul(T_IMG_CAM, np.matmul(T_CAM_Cuboid, back_left_bottom));


    b_l_t = np.matmul(T_IMG_CAM, np.matmul(T_CAM_Cuboid, back_left_top));


    # Make sure the
    # Remove z
    f_r_b_coord = (int(f_r_b[0][3] / f_r_b[2][3]), int(f_r_b[1][3] / f_r_b[2][3]));  # x, y coordinates in image
    f_r_t_coord = (int(f_r_t[0][3] / f_r_t[2][3]), int(f_r_t[1][3] / f_r_t[2][3]));
    f_l_b_coord = (int(f_l_b[0][3] / f_l_b[2][3]), int(f_l_b[1][3] / f_l_b[2][3]));
    f_l_t_coord = (int(f_l_t[0][3] / f_l_t[2][3]), int(f_l_t[1][3] / f_l_t[2][3]));



    b_r_b_coord = (int(b_r_b[0][3] / b_r_b[2][3]), int(b_r_b[1][3] / b_r_b[2][3]));
    b_r_t_coord = (int(b_r_t[0][3] / b_r_t[2][3]), int(b_r_t[1][3] / b_r_t[2][3]));
    b_l_b_coord = (int(b_l_b[0][3] / b_l_b[2][3]), int(b_l_b[1][3] / b_l_b[2][3]));
    b_l_t_coord = (int(b_l_t[0][3] / b_l_t[2][3]), int(b_l_t[1][3] / b_l_t[2][3]));



    #print(f_r_t_coord)
    '''

    c1 = (int(x_min[i]),int(y_min[i]))
    c2 = (int(x_max[i]), int(y_min[i]))
    c3 = (int(x_max[i]), int(y_max[i]))
    c4 = (int(x_min[i]), int(y_max[i]))

    cv2.line(img, c1, c2, [0, 0, 255], thickness=2, lineType=8, shift=0)
    cv2.line(img, c2, c3, [0, 0, 255], thickness=2, lineType=8, shift=0)
    cv2.line(img, c3, c4, [0, 0, 255], thickness=2, lineType=8, shift=0)
    cv2.line(img, c4, c1, [0, 0, 255], thickness=2, lineType=8, shift=0)
    '''

    cv2.line(img, f_r_b_coord, f_r_t_coord, [0, 0, 255], thickness=2, lineType=8, shift=0);
    cv2.line(img, f_r_b_coord, f_l_b_coord, [0, 0, 255], thickness=2, lineType=8, shift=0);
    cv2.line(img, f_l_b_coord, f_l_t_coord, [0, 0, 255], thickness=2, lineType=8, shift=0);
    cv2.line(img, f_l_t_coord, f_r_t_coord, [0, 0, 255], thickness=2, lineType=8, shift=0);
    # back
    cv2.line(img, b_r_b_coord, b_r_t_coord, [0, 0, 100], thickness=2, lineType=8, shift=0);
    cv2.line(img, b_r_b_coord, b_l_b_coord, [0, 0, 100], thickness=2, lineType=8, shift=0);
    cv2.line(img, b_l_b_coord, b_l_t_coord, [0, 0, 100], thickness=2, lineType=8, shift=0);
    cv2.line(img, b_l_t_coord, b_r_t_coord, [0, 0, 100], thickness=2, lineType=8, shift=0);
    # connect front to back
    cv2.line(img, f_r_b_coord, b_r_b_coord, [0, 0, 150], thickness=2, lineType=8, shift=0);
    cv2.line(img, f_r_t_coord, b_r_t_coord, [0, 0, 150], thickness=2, lineType=8, shift=0);
    cv2.line(img, f_l_b_coord, b_l_b_coord, [0, 0, 150], thickness=2, lineType=8, shift=0);
    cv2.line(img, f_l_t_coord, b_l_t_coord, [0, 0, 150], thickness=2, lineType=8, shift=0);
    

    #cv2.imshow('image', img)
    #cv2.waitKey(10000)



    lidar_path = "/media/wavelab/d3cd89ab-7705-4996-94f3-01da25ba8f50/autonomoose/" + path_type + "/filtered_lidar_points/data/filtered_" + format(frame, '010') + ".bin"  # change this to snowy or not snowy


    scan_data = np.fromfile(lidar_path, dtype=np.float32)  # numpy from file reads binary file
    lidar = scan_data.reshape((-1, 4))

    [rows, cols] = lidar.shape;  # rows and cols of the lidar points, rows = number of lidar points, columns (4 = x , y, z , intensity)

    print("YIP")
    print(lidar.shape)



    # 0: front , 1: front right, 2: right front, 3: back right, 4: back, 5: left back, 6: left front, 7: front left

    lidar_im_bag = []



    for i in range(rows):  # iterate to every lidar point (all the rows)

    # print(lidar[i,0:3]) #0,1,2

        p = np.array([0.0, 0.0, 0.0, 1.0]);  # set p to be a np array
        p[0:3] = lidar[i, 0:3]  # p[x,y,z] = lidar[x,y,z]
        # results in p[x,y,z,1]


        #p_img = np.matmul(T_IMG_LIDAR, p.transpose())  # we bring our points into the cuboid frame

        p_cuboid = np.matmul(T_Cuboid_LIDAR, p.transpose())

        p_c_x = p_cuboid[0]
        p_c_y = p_cuboid[1]
        p_c_z = p_cuboid[2]



        if (p_c_x< front_right_bottom[0][3] and p_c_x  > front_left_bottom[0][3] and p_c_y >
            front_right_top[1][3] and p_c_y < front_right_bottom[1][3] and p_c_z  <
            front_right_bottom[2][3] and p_c_z  > back_right_bottom[2][3]):
            #the above checks if the lidar point in cuboid frame, falls within the tracklet

                p_img = np.matmul(T_IMG_Cuboid, p_cuboid.transpose())
                # if our point cloud falls within the tracklet in the cuboid frame, we shall keep track of the number of point cloud points that do so
                lidar_im_bag.append(p_img)




    for j in range(len(lidar_im_bag)):



        p_x = int(lidar_im_bag[j][0] / lidar_im_bag[j][2])
        p_y = int(lidar_im_bag[j][1] / lidar_im_bag[j][2])



        cv2.circle(img, (p_x, p_y), radius, [255, 153, 255], thickness=2, lineType=8, shift=0)








cv2.imshow('image', img)
cv2.waitKey(10000)

