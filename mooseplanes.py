import numpy as np
import load_calibration
import os.path

calib_path = '/media/wavelab/d3cd89ab-7705-4996-94f3-01da25ba8f50/autonomoose/calibmoose';
calib = load_calibration.load_calibration(calib_path);

#note this is in the camera frame

def estimate_plane_coeffs(points):
    """Calculates least squares fit of a plane on a set of points and returns
    plane coefficients
    Args:
        points: points (N, 3)
    Returns:
        plane coefficients
    """

    #a plane is described by a normal vector n = [a,b,c] and a distance d
    #we have ax + by + cz + d = 0, let c =1
    all_points = np.vstack((points, [0.0, 1.653, 0.0]))
    centroid = np.mean(all_points, axis=0) #center of x, y, z
    shifted = points - centroid

    #if all points are relative to the centroid, then the plane runs through the origin
    points_x = shifted[:, 0]
    points_y = shifted[:, 1]
    points_z = shifted[:, 2]

    # Sums
    xx = np.dot(points_x, points_x)
    xy = np.dot(points_x, points_y)
    xz = np.dot(points_x, points_z)
    yy = np.dot(points_y, points_y)
    yz = np.dot(points_y, points_z)
    zz = np.dot(points_z, points_z)

    #cramer's rule

    det_x = yy * zz - yz * yz
    det_y = xx * zz - xz * xz
    det_z = xx * yy - xy * xy

    det_max = max(det_x, det_y, det_z)

    if det_max == det_x:
        normal = [det_x, xz * yz - xy * zz, xy * yz - xz * yy]
    elif det_max == det_y:
        normal = [xz * yz - xy * zz, det_y, xy * xz - yz * xx]
    else:
        normal = [xy * yz - xz * yy, xy * xz - yz * xx, det_z]


    normal = -(normal / np.linalg.norm(normal))
    d = -(normal[0] * centroid[0] +
          normal[1] * centroid[1] +
          normal[2] * centroid[2])

    return np.hstack([normal, d])


def estimate_ground_plane(point_cloud):
    """Estimates a ground plane by subsampling 2048 points in an area in front
    of the car, and running a least squares fit of a plane on the lowest
    points along y.
    Args:
        point_cloud: point cloud (3, N)
    Returns:
        ground_plane: ground plane coefficients
    """

    if len(point_cloud) == 0:
        raise ValueError('Lidar points are completely empty')



    ground_plane =0


    # Subsamples points in from of the car, 10m across and 30m in depth
    points = point_cloud[:,0:3]
    #points[:,1] = point_cloud[:,1] + 0.3

    all_points_near = points[
        (points[:, 0] > -5.0) &
        (points[:, 0] < 5.0) &
        (points[:, 2] < 30.0)]

    #these are the points near the car
    #x is 10m cross, z is 30 m forward


    if len(all_points_near) == 0:
        raise ValueError('No Lidar points in this frame')

    # Subsample near points
    subsample_num_near = 2048
    # gets 2048 random indices from all the near points
    near_indices = np.random.randint(0, len(all_points_near),
                                     subsample_num_near)




    points_subsampled = all_points_near[near_indices] #these are the random near points


    # Split into distance bins
    all_points_in_bins = []
    all_cropped_points = []
    for dist_bin_idx in range(3): #depth bins
        points_in_bin = points_subsampled[
            (points_subsampled[:, 2] > dist_bin_idx * 10.0) &
            (points_subsampled[:, 2] < (dist_bin_idx + 1) * 10.0)]
        #eg if dist_bin_idx = 0, then points_in_bin = z > 0, z < 10
        #if dist_bin_idx = 1, then points_in_bin = z > 10, z <20



        # Save to points in bins
        all_points_in_bins.extend(points_in_bin)

        # Sort by y for cropping

        y_order = np.argsort(points_in_bin[:, 1]) #returns indices that would sort the array, therefore indices starting from lowest y

        # note that y is the height, x is the width and z is the depth

        # Crop each bin
        num_points_in_bin = len(points_in_bin) #points between a z range (e.g z > 0 and z < 10)
        # crop_start_time = time.time()

        crop_indices = np.array([int(num_points_in_bin * 0.90),
                                 int(num_points_in_bin * 0.98)])
        #crop indices : slightly less than the number of points in the bin e.g number of points in the bin =96
        #crop indices : [86 94]




        #now we are only a few points in the bin with looking at the points in the bin (slightly more and slightly less)

        points_cropped = points_in_bin[
            y_order[crop_indices[0]:crop_indices[1]]] #picks a range of high y values, note that the higher the y the closer to ground because in the camera frame
            #y points downwards


        all_cropped_points.extend(points_cropped)

    all_cropped_points = np.asarray(all_cropped_points) #all the low y points from each bin



    # Do least squares fit to get ground plane coefficients
    ground_plane = estimate_plane_coeffs(all_cropped_points)
    


    return ground_plane



def create_planes_directory():
    home_dir ="/media/wavelab/d3cd89ab-7705-4996-94f3-01da25ba8f50/autonomoose/processedmoose"
    #current_dir ="%s/annotation" %pwd
    dirName ="%s/filtered_planes" %(home_dir)
    if not os.path.exists(dirName):
        os.makedirs(dirName)
        print("Directory " , dirName ,  " Created ")
    else:
        print("Directory " , dirName ,  " already exists")

    return dirName






def lidar_to_camera(lidar_path,camera): #lidar to front camera
    scan_data = np.fromfile(lidar_path, dtype=np.float32)  # numpy from file reads binary file
    lidar = scan_data.reshape((-1, 4))

    [rows,cols] = lidar.shape;  # rows and cols of the lidar points, rows = number of lidar points, columns (4 = x , y, z , intensity)

    # 0: front , 1: front right, 2: right front, 3: back right, 4: back, 5: left back, 6: left front, 7: front left


    point_cloud_cam = np.zeros([lidar.shape[0], lidar.shape[1]])

    if camera == 0:

        T_CAM_LIDAR = np.linalg.inv(np.array(calib['extrinsics']['T_LIDAR_CAM00'])); #going from lidar to camera

    elif camera ==1:
        T_CAM_LIDAR = np.linalg.inv(np.array(calib['extrinsics']['T_LIDAR_CAM01']))

    elif camera ==2:
        T_CAM_LIDAR = np.linalg.inv(np.array(calib['extrinsics']['T_LIDAR_CAM02']))

    elif camera ==3:
        T_CAM_LIDAR = np.linalg.inv(np.array(calib['extrinsics']['T_LIDAR_CAM03']))

    elif camera ==4:
        T_CAM_LIDAR = np.linalg.inv(np.array(calib['extrinsics']['T_LIDAR_CAM04']))
    elif camera ==5:
        T_CAM_LIDAR = np.linalg.inv(np.array(calib['extrinsics']['T_LIDAR_CAM05']))

    elif camera ==6:
        T_CAM_LIDAR = np.linalg.inv(np.array(calib['extrinsics']['T_LIDAR_CAM06']))

    elif camera ==7:
        T_CAM_LIDAR = np.linalg.inv(np.array(calib['extrinsics']['T_LIDAR_CAM07']))

    else:



    #print(T_CAM_LIDAR[:,1])

    for i in range(rows):
        point_cloud_cam[i] = np.matmul(T_CAM_LIDAR, lidar[i])


    return point_cloud_cam



#def lidar_to_camera_zero(point_cloud):


#print("This is the ground plane")
#print(estimate_ground_plane(lidar_to_camera_zero(lidar_path)))


def generate_planes():
    dirName = create_planes_directory()

    DISTORTED = False
    #frame = 0
    if DISTORTED:
        path_type = 'raw'
    else:
        path_type = 'processedmoose'

    lidar_path = "/media/wavelab/d3cd89ab-7705-4996-94f3-01da25ba8f50/autonomoose/" + path_type + "/filtered_lidar_points/data/"

    file_names = sorted(os.listdir(lidar_path))

    for a in range(8):

        for i in range(len(file_names)):
        #for i in range(1):
            frame = i
            file = lidar_path + file_names[i]

            if a == 0:
                frame_id = frame
            elif a == 1:
                frame_id = frame + 100

            elif a ==2:
                frame_id = frame + 200

            elif a ==3:
                frame_id = frame + 300

            elif a ==4:
                frame_id = frame + 400

            elif a ==5:
                frame_id = frame + 500
            elif a==6:
                frame_id = frame + 600
            elif a ==7:
                frame_id = frame +700


            point_cloud_cam = lidar_to_camera(file,a)

            coeff = estimate_ground_plane(point_cloud_cam)


            f = open(dirName +"/" + format(frame_id, '010') + ".txt", "w+")

            f.write("%s %s %s %s" %(coeff[0], coeff[1], coeff[2], coeff[3]))
            f.close()



generate_planes()



