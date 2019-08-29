#load calibration.py has all the calbration matrices

#!/usr/bin/env python

'''
want only annotations that have at least 20 lidar points
writes out annotation files
'''
import sys
#print(sys.path)

sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import json
import numpy as np
import cv2
import load_calibration
from scipy.spatial.transform import Rotation as R


#Create target Directory if don't exist
import os.path


def create_annotation_directory(camera):
    home_dir ="/media/wavelab/d3cd89ab-7705-4996-94f3-01da25ba8f50/autonomoose/processedmoose"
    #current_dir ="%s/annotation" %pwd
    dirName ="%s/filtered_annotation_0%s" %(home_dir,camera)
    if not os.path.exists(dirName):
        os.makedirs(dirName)
        print("Directory " , dirName ,  " Created ")
    else:
        print("Directory " , dirName ,  " already exists")

    return dirName






def write_txt_annotation(frame,cam,directory):



    cam = str(cam)
    DISTORTED = False

    writeout = True
    filter = 20 #only show/write out annotations that have at least 10 lidar points in them

    img_path = "/media/wavelab/d3cd89ab-7705-4996-94f3-01da25ba8f50/autonomoose/processedmoose/image_0" + cam + "/data/" + format(frame, '010') + ".png"
    print(img_path)
    if DISTORTED:
      path_type = 'raw'
    else:
      path_type = 'processedmoose'
    annotations_file = '/media/wavelab/d3cd89ab-7705-4996-94f3-01da25ba8f50/autonomoose/3d_annotations.json'; #contains all the annotations
    lidar_path = "/media/wavelab/d3cd89ab-7705-4996-94f3-01da25ba8f50/autonomoose/" + path_type + "/lidar_points/data/" + format(frame, '010') + ".bin" #change this to snowy or not snowy
    calib_path = '/media/wavelab/d3cd89ab-7705-4996-94f3-01da25ba8f50/autonomoose/calibmoose'
    calib = load_calibration.load_calibration(calib_path);


    # Load 3d annotations
    annotations_data = None
    with open(annotations_file) as f:
        annotations_data = json.load(f)




    # Projection matrix from camera to image frame
    T_IMG_CAM = np.eye(4); #identity matrix
    T_IMG_CAM[0:3,0:3] = np.array(calib['CAM0' + cam]['camera_matrix']['data']).reshape(-1, 3); #camera to image #this comes from e.g "F.yaml" file

    #T_IMG_CAM : 4 x 4 matrix
    T_IMG_CAM = T_IMG_CAM[0:3,0:4]; # remove last row, #choose the first 3 rows and get rid of the last row

    T_CAM_LIDAR = np.linalg.inv(np.array(calib['extrinsics']['T_LIDAR_CAM0' + cam])); #going from lidar to camera

    T_IMG_LIDAR = np.matmul(T_IMG_CAM, T_CAM_LIDAR); # go from lidar to image

    img = cv2.imread(img_path)
    img_h, img_w = img.shape[:2] #get the image height and width



    #Add each cuboid to image
    #the coordinates in the tracklet json are lidar coords


    if cam =='0':
        frame_id = frame
    elif cam =='1':
        frame_id = frame + 100
    elif cam == '2':
        frame_id = frame + 200
    elif cam == '3':
        frame_id = frame + 300

    elif cam =='4':
        frame_id = frame + 400
    elif cam =='5':
        frame_id = frame + 500
    elif cam =='6':
        frame_id = frame + 600
    else:
        frame_id = frame + 700


    f = open(directory + "/"+ format(frame_id, '010') + ".txt", "w+")
    for cuboid in annotations_data[frame]['cuboids']:


      T_Lidar_Cuboid = np.eye(4); #identity matrix
      T_Lidar_Cuboid[0:3,0:3] = R.from_euler('z', cuboid['yaw'], degrees=False).as_dcm(); #make a directional cosine matrix using the yaw, i.e rotation about z, yaw angle

      '''
      Rotations in 3 dimensions can be represented by a sequece of 3 rotations around a sequence of axes. 
      In theory, any three axes spanning the 3D Euclidean space are enough. In practice the axes of rotation are chosen to be the basis vectors.
    
      The three rotations can either be in a global frame of reference (extrinsic) or in a body centred frame of refernce (intrinsic),
      which is attached to, and moves with, the object under rotation
      
      In our case, 'z' specifies the axis of rotation (extrinsic rotation), cudoid['yaw] euler angles in radians (rotation angle)
      Returns object containing the rotation represented by the sequencce of rotations around given axes with given angles
      
      T_Lidar_Cuboid = basic rotation (elemental rotation) : R_z(theta = yaw) = [[ cos(theta) - sin(theta) 0 etc]]
      * .as_dcm - Represent as direction cosine matrices.
    
      3D rotations can be represented using direction cosine matrices, which are 3 x 3 real orthogonal matrices with determinant equal to +1
      
      '''

      T_Lidar_Cuboid[0][3] = cuboid['position']['x']; #center of the tracklet, from cuboid to lidar
      T_Lidar_Cuboid[1][3] = cuboid['position']['y'];
      T_Lidar_Cuboid[2][3] = cuboid['position']['z'];

      #the above is the translation part of the transformation matrix, so now we have the rotation and the translation

      T_Cuboid_Lidar = np.linalg.inv(T_Lidar_Cuboid) #go from Lidar to Cuboid frame


      width = cuboid['dimensions']['x']; #x is just a naming convention that scale uses, could have easily been cuboid['dimensions']['width']
      length = cuboid['dimensions']['y'];
      height = cuboid['dimensions']['z'];
      radius = 3

      #note that in the lidar frame, up is z, forwards is x, side is y
      # Create circle in middle of the cuboid


      T_CAM_CUBOID= np.matmul(T_CAM_LIDAR, T_Lidar_Cuboid)  #going from cuboid to lidar to camera, i.e from cuboid to camera



      #this is a check, comment it
      #now we make a fake transformation that goes from Lidar to Camera, i.e 90 degrees around z, 90 degrees about x
      #note if we rotate the lidar frame to the camera frame, we will go -90 in z, - 90 in x
      #since we are rotating the vector, we will + 90 z and + 90 x
      #theta_x = np.pi/2 #rotating the vector
      #theta_z = np.pi/2 #rotating the vector
      #rx = np.array([[1, 0, 0], [0, np.cos(theta_x), -np.sin(theta_x)], [0, np.sin(theta_x), np.cos(theta_x)]])
      #rz = np.array([[np.cos(theta_z), -np.sin(theta_z), 0], [np.sin(theta_z), np.cos(theta_z), 0], [0, 0, 1]])

      #T_CAM_LIDAR_FAKE = np.matmul(rx,rz) #rotation 90 degree about z, then rotation 90 about x



      #print(np.matmul(T_CAM_LIDAR_FAKE,np.array([0,1,0])))

      #T_CAM_CUBOID_FAKE  = np.matmul(T_CAM_LIDAR_FAKE[0:3, 0:3], T_Lidar_Cuboid[0:3, 0:3])

      '''
      The LiDAR frame has x forward, y left and z up. The yaw should be around the z axis
      Camera frame has x to the right, y down and z forward.
      '''

      if T_CAM_CUBOID[2][3] < 0: # Behind camera #z value is neg, we shall move on with our lives
        continue;
      T_IMG_CUBOID = np.matmul(T_IMG_CAM, T_CAM_CUBOID); #lidarcuboid to image
      x = int(T_IMG_CUBOID[0][3]/T_IMG_CUBOID[2][3]);#x value/ z
      y = int(T_IMG_CUBOID[1][3]/T_IMG_CUBOID[2][3]); #y value/z

      '''
      Very import to remember that The LiDAR frame has x forward, y left and z up. The given yaw is around the z axis
      the cuboid frame which we create has the same axis
      '''
      front_right_bottom = np.array([[1,0,0,length/2],[0,1,0,-width/2],[0,0,1,-height/2],[0,0,0,1]]); #cuboid in cuboid coordinates
      front_right_top = np.array([[1,0,0,length/2],[0,1,0,-width/2],[0,0,1,height/2],[0,0,0,1]]);
      front_left_bottom = np.array([[1,0,0,length/2],[0,1,0,width/2],[0,0,1,-height/2],[0,0,0,1]]);
      front_left_top = np.array([[1,0,0,length/2],[0,1,0,width/2],[0,0,1,height/2],[0,0,0,1]]);

      back_right_bottom = np.array([[1,0,0,-length/2],[0,1,0,-width/2],[0,0,1,-height/2],[0,0,0,1]]);
      back_right_top = np.array([[1,0,0,-length/2],[0,1,0,-width/2],[0,0,1,height/2],[0,0,0,1]]);
      back_left_bottom = np.array([[1,0,0,-length/2],[0,1,0,width/2],[0,0,1,-height/2],[0,0,0,1]]);
      back_left_top = np.array([[1,0,0,-length/2],[0,1,0,width/2],[0,0,1,height/2],[0,0,0,1]]);

      scan_data = np.fromfile(lidar_path, dtype=np.float32)  # numpy from file reads binary file
      lidar = scan_data.reshape((-1, 4))

      [rows,cols] = lidar.shape;  # rows and cols of the lidar points, rows = number of lidar points, columns (4 = x , y, z , intensity)

      # 0: front , 1: front right, 2: right front, 3: back right, 4: back, 5: left back, 6: left front, 7: front left
      lidar_bag = []

      for i in range(rows):  # iterate to every lidar point (all the rows)

          #print(lidar[i,0:3]) #0,1,2


          p = np.array([0.0, 0.0, 0.0, 1.0]);  # set p to be a np array
          p[0:3] = lidar[i, 0:3]  # p[x,y,z] = lidar[x,y,z]
          #results in p[x,y,z,1]



          p_cuboid = np.matmul(T_Cuboid_Lidar,p.transpose()) #we bring our points into the cuboid frame

          if (p_cuboid[0] < front_right_bottom[0][3] and p_cuboid[0] > back_right_bottom[0][3] and p_cuboid[1] > front_right_bottom[1][3] and p_cuboid[1] < front_left_bottom[1][3] and p_cuboid[2]  > front_right_bottom[2][3] and p_cuboid[2] < front_right_top[2][3] ):
              lidar_bag.append(i) #if our point cloud falls within the tracklet in the cuboid frame, we shall keep track of the number of point cloud points that do so

      if len(lidar_bag) > filter: #if we filter such that the annotations should have at least 20 lidar



          #print("These are the euler angles")
          #print(R.from_dcm(T_CAM_CUBOID[0:3, 0:3]).as_euler('zyx', degrees=True)) #the z coordinates will show how much we moved around the original z which happens
          #to be the same as the rotation around our new y axis, but we must account for the 90 degree x rotation and the definition of our positive direction
          #i.e rotation y = negative (R.from_dcm(T_CAM_CUBOID[0:3, 0:3]).as_euler('zyx', degrees=False)
          #same as : ry = -lidar - pi/2
          tmp_yaw = -R.from_dcm(T_CAM_CUBOID[0:3, 0:3]).as_euler('zyx', degrees=False)[0]




          # we want to ensure that the yaw is between -pi and pi
          if tmp_yaw < -np.pi:
              rotation_y = tmp_yaw + 2*np.pi

          elif tmp_yaw  > np.pi:
              rotation_y = tmp_yaw - 2 * np.pi

          else:
              rotation_y = tmp_yaw


          #print("This is the y rotation")
          #print(rotation_y/np.pi * 180)

          #print("This is the given yaw in lidar frame")
          #print (cuboid['yaw']/np.pi*180)

          viewing_angle = np.arctan2(T_CAM_CUBOID[0][3], T_CAM_CUBOID[2][3]) #arctan2(x/z)

          #print("This is the viewing angle")
          #print(viewing_angle/np.pi * 180)

          alpha_tmp = rotation_y - viewing_angle

          if alpha_tmp < -np.pi:
              alpha = alpha_tmp +  2*np.pi

          elif alpha_tmp > np.pi:
              alpha = alpha_tmp - 2*np.pi

          else:
              alpha = alpha_tmp

          #print("This is alpha")
          #print(alpha/np.pi*180)

          #print("***************************")


          cv2.circle(img, (x, y), radius, [0, 0, 255], thickness=2, lineType=8, shift=0)

          # Project to image
          f_r_b_cam = np.matmul(T_CAM_LIDAR, np.matmul(T_Lidar_Cuboid, front_right_bottom));  # cuboid to lidar, lidar to camera

          f_r_b = np.matmul(T_IMG_CAM, f_r_b_cam);  # camera to image

          f_r_t_cam = np.matmul(T_CAM_LIDAR, np.matmul(T_Lidar_Cuboid, front_right_top));


          f_r_t = np.matmul(T_IMG_CAM, f_r_t_cam);
          f_l_b_cam = np.matmul(T_CAM_LIDAR, np.matmul(T_Lidar_Cuboid, front_left_bottom));

          f_l_b = np.matmul(T_IMG_CAM, f_l_b_cam);
          f_l_t_cam = np.matmul(T_CAM_LIDAR, np.matmul(T_Lidar_Cuboid, front_left_top));

          f_l_t = np.matmul(T_IMG_CAM, f_l_t_cam);

          b_r_b_cam = np.matmul(T_CAM_LIDAR, np.matmul(T_Lidar_Cuboid, back_right_bottom));

          b_r_b = np.matmul(T_IMG_CAM, b_r_b_cam);

          b_r_t_cam = np.matmul(T_CAM_LIDAR, np.matmul(T_Lidar_Cuboid, back_right_top));


          b_r_t = np.matmul(T_IMG_CAM, b_r_t_cam);
          b_l_b_cam = np.matmul(T_CAM_LIDAR, np.matmul(T_Lidar_Cuboid, back_left_bottom));

          b_l_b = np.matmul(T_IMG_CAM, b_l_b_cam);
          b_l_t_cam = np.matmul(T_CAM_LIDAR, np.matmul(T_Lidar_Cuboid, back_left_top));

          b_l_t = np.matmul(T_IMG_CAM, b_l_t_cam);

          # Make sure the
          # Remove z
          f_r_b_coord = (int(f_r_b[0][3] / f_r_b[2][3]), int(f_r_b[1][3] / f_r_b[2][3]));  # x, y coordinates image coordinates to draw using opencv
          f_r_t_coord = (int(f_r_t[0][3] / f_r_t[2][3]), int(f_r_t[1][3] / f_r_t[2][3]));
          f_l_b_coord = (int(f_l_b[0][3] / f_l_b[2][3]), int(f_l_b[1][3] / f_l_b[2][3]));
          f_l_t_coord = (int(f_l_t[0][3] / f_l_t[2][3]), int(f_l_t[1][3] / f_l_t[2][3]));

          b_r_b_coord = (int(b_r_b[0][3] / b_r_b[2][3]), int(b_r_b[1][3] / b_r_b[2][3]));
          b_r_t_coord = (int(b_r_t[0][3] / b_r_t[2][3]), int(b_r_t[1][3] / b_r_t[2][3]));
          b_l_b_coord = (int(b_l_b[0][3] / b_l_b[2][3]), int(b_l_b[1][3] / b_l_b[2][3]));
          b_l_t_coord = (int(b_l_t[0][3] / b_l_t[2][3]), int(b_l_t[1][3] / b_l_t[2][3]));


          f_r_b_coord_im = (f_r_b[0][3] / f_r_b[2][3], f_r_b[1][3] / f_r_b[2][3]);  # x, y coordinates image coordinates
          f_r_t_coord_im = (f_r_t[0][3] / f_r_t[2][3], f_r_t[1][3] / f_r_t[2][3]);
          f_l_b_coord_im = (f_l_b[0][3] / f_l_b[2][3], f_l_b[1][3] / f_l_b[2][3]);
          f_l_t_coord_im = (f_l_t[0][3] / f_l_t[2][3], f_l_t[1][3] / f_l_t[2][3]);

          b_r_b_coord_im = (b_r_b[0][3] / b_r_b[2][3], b_r_b[1][3] / b_r_b[2][3]);
          b_r_t_coord_im = (b_r_t[0][3] / b_r_t[2][3], b_r_t[1][3] / b_r_t[2][3]);
          b_l_b_coord_im = (b_l_b[0][3] / b_l_b[2][3], b_l_b[1][3] / b_l_b[2][3]);
          b_l_t_coord_im = (b_l_t[0][3] / b_l_t[2][3], b_l_t[1][3] / b_l_t[2][3]);

          f_r_b_coord_cam = (f_r_b_cam[0][3] / f_r_b_cam[2][3], f_r_b_cam[1][3] / f_r_b_cam[2][3]);  # x, y coordinates camera coordinates
          f_r_t_coord_cam = (f_r_t_cam[0][3] / f_r_t_cam[2][3], f_r_t_cam[1][3] / f_r_t_cam[2][3]);
          f_l_b_coord_cam = (f_l_b_cam[0][3] / f_l_b_cam[2][3], f_l_b_cam[1][3] / f_l_b_cam[2][3]);
          f_l_t_coord_cam = (f_l_t_cam[0][3] / f_l_t_cam[2][3], f_l_t_cam[1][3] / f_l_t_cam[2][3]);

          b_r_b_coord_cam = (b_r_b_cam[0][3] / b_r_b_cam[2][3], b_r_b_cam[1][3] / b_r_b_cam[2][3]);
          b_r_t_coord_cam = (b_r_t_cam[0][3] / b_r_t_cam[2][3], b_r_t_cam[1][3] / b_r_t_cam[2][3]);
          b_l_b_coord_cam = (b_l_b_cam[0][3] / b_l_b_cam[2][3], b_l_b_cam[1][3] / b_l_b_cam[2][3]);
          b_l_t_coord_cam = (b_l_t_cam[0][3] / b_l_t_cam[2][3], b_l_t_cam[1][3] / b_l_t_cam[2][3]);


          #if x < 0 or x > img_w or y < 0 or y > img_h:  # if the center is outside the image range, continues to the next cycle in the for loop
              #continue;

          #if all of the bb corners are outside of the image
          if (f_r_b_coord_im[0] < 0 or f_r_b_coord_im[0] > img_w) and \
                  (f_r_t_coord_im[0] < 0 or f_r_t_coord_im[0] > img_w) and \
                  (f_l_b_coord_im[0] < 0 or f_l_b_coord_im[0] > img_w) and \
                  (f_l_t_coord_im[0] < 0 or f_l_t_coord_im[0] > img_w) and \
                  (b_r_b_coord_im[0] < 0 or b_r_b_coord_im[0] > img_w) and \
                  (b_r_t_coord_im[0] < 0 or b_r_t_coord_im[0] > img_w) and \
                  (b_l_b_coord_im[0] < 0 or b_l_b_coord_im[0] > img_w) and \
                  (b_l_t_coord_im[0] < 0 or b_l_t_coord_im[0] > img_w) or \
                  (f_r_b_coord_im[1] < 0 or f_r_b_coord_im[1] > img_h) and\
                  (f_r_t_coord_im[1] < 0 or f_r_t_coord_im[1] > img_h) and \
                  (f_l_b_coord_im[1] < 0 or f_l_b_coord_im[1] > img_h) and \
                  (f_l_t_coord_im[1] < 0 or f_l_t_coord_im[1] > img_h) and \
                  (b_r_b_coord_im[1] < 0 or b_r_b_coord_im[1] > img_h) and \
                  (b_r_t_coord_im[1] < 0 or b_r_t_coord_im[1] > img_h) and \
                  (b_l_b_coord_im[1] < 0 or b_l_b_coord_im[1] > img_h) and \
                  (b_l_t_coord_im[1] < 0 or b_l_t_coord_im[1] > img_h):

                  continue






          #Extract 2D BB left, top, right, bottom : xmin, ymin, xmax, y max
          #print(f_r_b_coord_cam[0])

          #we want the x min, ymin, xmax, ymax in image coordinates
          x_min = np.min([f_r_b_coord_im[0], f_r_t_coord_im[0],f_l_b_coord_im[0],f_l_t_coord_im[0],b_r_b_coord_im[0],b_r_t_coord_im[0],b_l_b_coord_im[0], b_l_t_coord_im[0]])
          y_min = np.min([f_r_b_coord_im[1], f_r_t_coord_im[1], f_l_b_coord_im[1], f_l_t_coord_im[1], b_r_b_coord_im[1], b_r_t_coord_im[1], b_l_b_coord_im[1], b_l_t_coord_im[1]])
          x_max=np.max([f_r_b_coord_im[0], f_r_t_coord_im[0],f_l_b_coord_im[0],f_l_t_coord_im[0],b_r_b_coord_im[0],b_r_t_coord_im[0],b_l_b_coord_im[0], b_l_t_coord_im[0]])
          y_max =np.max([f_r_b_coord_im[1], f_r_t_coord_im[1], f_l_b_coord_im[1], f_l_t_coord_im[1], b_r_b_coord_im[1], b_r_t_coord_im[1],b_l_b_coord_im[1], b_l_t_coord_im[1]])

          if (x_min < 0 and x_max > img_w): #get rid of any weird huge bb, where x min and x max span over the whole image
              continue

          if (y_min < 0 and y_max > img_h): #get rid of any huge bb where y min and y max span over the whole image
              continue



          #truncation calculation

          if x_min < 0:
              x_min_set = 0

          elif x_min > img_w:
              x_min_set = img_w

          else:
              x_min_set = x_min

          if y_min < 0:
              y_min_set = 0

          elif y_min > img_h:
              y_min_set = img_h

          else:
              y_min_set = y_min

          if x_max > img_w:
              x_max_set = img_w

          elif x_max < 0:
              x_max_set =0
          else:
              x_max_set = x_max

          if y_max > img_h:
              y_max_set = img_h

          elif y_max < 0:
              y_max_set = 0
          else:
              y_max_set = y_max


          area_actual = (x_max - x_min) * (y_max - y_min)
          area_set = (x_max_set - x_min_set)* (y_max_set - y_min_set)



          if area_set ==0: #tracklet is outside of the image
              continue

          ratio_of_area = area_set/area_actual

          if ratio_of_area == 1:
              truncation = 0

          else:
              truncation = 1 - ratio_of_area






          # Draw  12 lines
          # Front
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
          '''

          #print(cuboid)

          camera_x_center = T_CAM_CUBOID[0][3]
          camera_y_ground = T_CAM_CUBOID[1][3] + height/2 #note that this is how the kitti label is, the y is on the ground
          camera_z_center = T_CAM_CUBOID[2][3]

          '''
          example of a cuboid statement: {'uuid': '33babea4-958b-49a1-ac65-86174faa111a', 'attributes': {'state': 'Moving'}, 'dimensions': {'y': 4.276, 'x': 1.766, 'z': 1.503}, 'label': 'Car', 'position': {'y': 5.739311373648604, 'x': 57.374972338211876, 'z': -1.5275162154592332}, 'camera_used': None, 'yaw': -3.1134003618947323, 'stationary': False}
          '''

          f.write(cuboid['label'])
          f.write(' %s '%(round(truncation,2))) #trucation

          f.write('0 ') #occlusion
          f.write('%s ' %(round(alpha,2)))
          f.write('%s %s %s %s ' %(round(x_min_set,2), round(y_min_set,2), round(x_max_set,2), round(y_max_set,2))) #pixel
          f.write('%s %s %s ' %(round(height,2), round(width,2), round(length,2)))#length, width, height
          f.write('%s %s %s ' %(round(camera_x_center,2),round(camera_y_ground,2),round(camera_z_center,2))) #center of tracklet in camera coordinates
          f.write('%s ' %(round(rotation_y,2)))
          f.write('\n')
    f.close()

    #cv2.imshow('image',img)
    #cv2.waitKey(10000)
    #cv2.imwrite("image1.jpg")


def write_all_annotations(cam):
    cam = str(cam)
    directory = create_annotation_directory(cam)
    img_dir = "/media/wavelab/d3cd89ab-7705-4996-94f3-01da25ba8f50/autonomoose/processedmoose/image_0" + cam + "/data"
    file_names = sorted(os.listdir(img_dir))
    #print(file_names)

    for frame in range(len(file_names)):
        write_txt_annotation(frame,cam,directory)


for cam in range(8):



    cam = str(cam)
    write_all_annotations(cam)