# Tools for the Canadian Adverse Driving Conditions Dataset (CADCD)

Credit: TraiLab 

In this repo, we have tools for the CADCD. The CADCD was created by researchers from the University of Toronto Artificial Intelligence Laboratory (TRAILab) and 
the University of Waterloo Intelligent SystemsEngineering Lab (WISELab). The data was gathered using the 
Autonomoose (https://www.autonomoose.net) in Waterloo during the winter thus capturing adverse weather conditions. 
The 3D annotations of objects were done by ensuring that each tracklet was covered by at least five lidarpoints. 
The annotations were made by expert annotators at Scale (https://scale.com/) and validation steps are currently ongoing.
The occlusion information of the objects was not included. A full CADCD datatset comprising of 10,000 frames
at 10Hz will be released in the future. 





![alt text](https://github.com/asvath/mobile_robotics/blob/master/final%20results/3d.png)
![alt text](https://github.com/asvath/mobile_robotics/blob/master/final%20results/2dbb.png)

## filtermoosekitt.py : Write out the CADCD annotations into KITTI Format

We convert the CADCD json annotations into the the KITTI format. Only tracklets that have a minimum of 20 lidar points were 
considered for the conversion of the CADCD annotations to the KITTI format as part of ground truth.
For each tracklet, the script calculates the truncation, observation angle, pixel coordinates of the 2D bounding box from the given 3D bounding box, and the rotation about the Y-
axis of the camera from the given yaw angle about the lidar sensor. As the occlusion information was not provided, we set
occlusion to ‘0: fully visible’ in the ground truth labels. This is to ensure that the annotations can be split into ‘Easy’, 
‘Moderate’ and ‘Hard’ categories according to the KITTI standard.

Fig 2. shows an example of
the annotations before and after running the script. 
