#!/home/wavelab/.virtualenvs/moose2ros/bin/python2
# -*- coding: utf-8 -*-

import os
import rospy
import rosbag
from datetime import datetime
from std_msgs.msg import Header
from sensor_msgs.msg import CameraInfo, Imu, PointField, NavSatFix
import sensor_msgs.point_cloud2 as pcl2
import numpy as np
import argparse
import progressbar



def save_velo_data(bag, directory ,velo_frame_id, topic):  # velo_frame_id = 'lidar' velo_topic = '/autonomoose/lidar'
    print("Exporting velodyne data")



    #velo_path = os.path.join(directory, 'processedmoose/lidar_points')  # path to the velodyne points
    velo_path = os.path.join(directory, '2019_8_20_12_55_44_multiplier_4')  # path to the velodyne points

    #velo_data_dir = os.path.join(velo_path, 'data')  # to where the lidar points live
    velo_data_dir = os.path.join(velo_path, 'filtered')

    velo_filenames = sorted(os.listdir(velo_data_dir))  # get all of them

    with open(os.path.join(velo_path, 'timestamps.txt')) as f:  # get the timestamps
        lines = f.readlines()  # going line by line
        velo_datetimes = []
        for line in lines:
            if len(line) == 1:
                continue
            dt = datetime.strptime(line[:-4], '%Y-%m-%d %H:%M:%S.%f')  # formatting the time
            print(dt)
            velo_datetimes.append(dt)
    iterable = zip(velo_datetimes, velo_filenames)

    bar = progressbar.ProgressBar()
    for dt, filename in bar(iterable):
        if dt is None:


            continue

        #iterable = zip(velo_datetimes, velo_filenames)  # now we have the times and the filenames

        velo_filename = os.path.join(velo_data_dir, filename)
        print(velo_filename)
        # read binary data
        scan = (np.fromfile(velo_filename, dtype=np.float32)).reshape(-1, 4)  # x y z reflectance
        #print(scan)

        # create header
        header = Header()
        header.frame_id = velo_frame_id
        header.stamp = rospy.Time.from_sec(float(datetime.strftime(dt, "%s.%f")))

        # fill pcl msg
        fields = [PointField('x', 0, PointField.FLOAT32, 1),
                    PointField('y', 4, PointField.FLOAT32, 1),
                    PointField('z', 8, PointField.FLOAT32, 1),
                    PointField('intensity', 12, PointField.FLOAT32, 1)]
        pcl_msg = pcl2.create_cloud(header, fields, scan)
        #print(scan)
        #print(pcl_msg)

        print(pcl_msg.header.stamp)
        bag.write(topic + '/pointcloud', pcl_msg, t=pcl_msg.header.stamp)


def main():
    parser = argparse.ArgumentParser(description="Convert KITTI dataset to ROS bag file the easy way!")
    # Accepted argument values

    #parser.add_argument("dir", nargs="?", default='/media/wavelab/d3cd89ab-7705-4996-94f3-01da25ba8f50/autonomoose',
                        #help="base directory of the dataset, if no directory passed the deafult is current working directory")

    parser.add_argument("dir", nargs="?", default='/home/wavelab/snow_removal_results/moose',
                        help="base directory of the dataset, if no directory passed the deafult is current working directory")

    parser.add_argument("-f", "--frame",
                        help="frame number of the raw dataset (i.e. 000000001) or all to do everything ")

    args = parser.parse_args()

    compression = rosbag.Compression.NONE








    bag = rosbag.Bag("filtered_autonomoose_multiplier_4.bag", 'w',    compression=compression)

    try:

        velo_frame_id = 'velo_link'
        #frame ids are simply strings which uniquely identify coordinate frames
        velo_topic = '/autonomoose/velo'
        #topic is a name that is used to identify the content of the message



        save_velo_data(bag, args.dir,velo_frame_id, velo_topic)  # this is the only one that is needed

    finally:
        print("## OVERVIEW ##")
        print(bag)
        bag.close()





if __name__ == '__main__':
    main()

