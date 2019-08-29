# Tools for the Canadian Adverse Driving Conditions Dataset (CADCD)

Credit: TraiLab 

In this repo, we have tools for the CADCD. The CADCD was created by researchers from the University of Toronto Artificial Intelligence Laboratory (TRAILab) and 
the University of Waterloo Intelligent Systems Engineering Lab (WISELab). The data was gathered using the 
Autonomoose (https://www.autonomoose.net) in Waterloo during the winter thus capturing adverse weather conditions. 
The 3D annotations of objects were done by ensuring that each tracklet was covered by at least five lidarpoints. 
The annotations were made by expert annotators at Scale (https://scale.com/) and validation steps are currently ongoing.
The occlusion information of the objects was not included. A full CADCD datatset comprising of 10,000 frames
at 10Hz will be released in the future. 

### Folder structure
To utilize the files in this repo, we recommend the following folder structure:
	
	autonomoose/
	└── calibmoose/
	│	└── F.yaml, B.yaml etc
	│
	└──devmoose/
	│	└── filterkittimoose.py, etc (all files from the repo)
	│
	└──processedmoose/
	│	└── image_0[0-7]/
	│	│	└── 0000000001.png
	│	└── lidar_points/
	│		└── data/
	│		│	└── 0000000001.bin
	│     		└── timestamps.txt
	│			
	└── 3d_annotations.json			
	

### Write out the CADCD annotations into KITTI Format : filtermoosekitti.py 

We convert the CADCD json annotations into the the KITTI format. Only tracklets that have a minimum of 20 lidar points were 
considered for the conversion of the CADCD annotations to the KITTI format as part of ground truth.
For each tracklet, the script calculates the truncation, observation angle, pixel coordinates of the 2D bounding box from the given 3D bounding box, and the rotation about the Y-axis of the camera from the given yaw angle about the lidar sensor. 
As the occlusion information was not provided, we set occlusion to ‘0: fully visible’ in the ground truth labels. 
This is to ensure that the annotations can be split into ‘Easy’, ‘Moderate’ and ‘Hard’ categories according to the KITTI standard.

Note that CADCD has 8 cameras and the sample that we worked on consists of 800 frames; 100 from each frame. We do not have a reference camera (KITTI has a reference camera 0). Our script outputs the annotations for each frame wrt to the camera that it comes from. The output files have names that reflect the camera. E.g CAM 0: 0000000001.txt, whereas CAM 7: 0000000701.txt. The files will be in output directories, e.g annotation_00, annotation_07 etc.


	Example:
	$python filtermoosekitti.py

There is an example of the output annotation file with follows the kitti format:


	Example:
	Car 0 0 1.68 558.02 504.09 585.82 527.24 1.76 1.8 3.9 -5.47 2.85 52.91 1.57 
	Car 0.07 0 0.7 0 515.22 105.06 546.83 1.51 1.79 4.13 -30.47 2.81 34.05 -0.03 
	Car 0 0 -2.44 101.5 507.98 194.65 538.23 1.79 2.0 4.36 -30.53 2.89 40.97 -3.09 
	
Note that the y center coordinates is on the ground instead of the spatial center of the tracketlet. The x and z coordinates are in the spatial center. The format for KITTI labels is explained in the readme.txt from the "Object Development Kit".


The figure below shows an example of the annotations before and after running the script. 


![alt text](https://github.com/asvath/cadcd/blob/master/pics/annotations_before_conversion.png)
![alt text](https://github.com/asvath/cadcd/blob/master/pics/annotations_after_conversion.png)

### Visualize the CADCD annotations in KITTI Format with lidar points : visualize_kitti_format_tracklet_with_lidar_points.py

The script enables us to visualize the KITTI format CADCD annotations. This includes visualizing the lidar points that "belong" to the tracklets. This can be edited to only show the tracklets. The figure below shows an example of a visualization with the lidar points.


<img src="https://github.com/asvath/cadcd/blob/master/pics/tracklets_w_lidar.png" width="520" height="400">




### Visualize the CADCD annotations in birds eye view with lidar points : birds_eye_view.py

The script enables us to visualize the CADCD annotations in birds eye view (BEV) with lidar points. The figure below shows an example of the annotations in BEV.

<img src="https://github.com/asvath/cadcd/blob/master/pics/moose_bev_11.png" width="500" height="500">


### Generate ground planes : mooseplanes.py
To generate ground planes for CADCD, we utilized the estimate_plane_coeffs and estimate_ground_plane functions from AVOD’s ROS integration package. The lidar point cloud was first converted into each camera frame. The ground planes were then generated by subsampling 2048 lidar points in a 5m x 30m area infront of the camera. Least squares fit of a plane was performed on the lowest points along the y-axis. As the functions randomly subsample 2048 lidar points, we can acquire different ground planes for the same data. We should probably improve on this code. We could consider using RANSC. 

### Write out the CADCD binary lidar files into a single ROS bag : moose2fullbag.py 
We can convert the CADCD lidar binary format files (.bin) to a single rosbag (.bag) format file by utilizing the moose2bag.py script. It was made by modifying the publicly available kitti2bag.py script
(https://github.com/tomas789/kitti2bag).

### Convert text files with lidar info into binary files : text2binary.py

We can convert text files with lidar info into binary files by utilizing the text2binary.py script

There is an example of the lidar text file  :

	Example:
	5.942 0.3772 -2.259 0.01961
	-69.89 5.341 -1.975 0.07059
	-9.1 0.2299 -2.233 0.03922
	-12.19 0.8624 -2.216 0.03922
	



