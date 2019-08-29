# Tools for the Canadian Adverse Driving Conditions Dataset (CADCD)

Credit: TraiLab 

In this repo, we have tools for the CADCD. The CADCD was created by researchers from the University of Toronto Artificial Intelligence Laboratory (TRAILab) and 
the University of Waterloo Intelligent SystemsEngineering Lab (WISELab). The data was gathered using the 
Autonomoose (https://www.autonomoose.net) in Waterloo during the winter thus capturing adverse weather conditions. 
The 3D annotations of objects were done by ensuring that each tracklet was covered by at least five lidarpoints. 
The annotations were made by expert annotators at Scale (https://scale.com/) and validation steps are currently ongoing.
The occlusion information of the objects was not included. A full CADCD datatset comprising of 10,000 frames
at 10Hz will be released in the future. 


### Write out the CADCD annotations into KITTI Format : filtermoosekitti.py 

We convert the CADCD json annotations into the the KITTI format. Only tracklets that have a minimum of 20 lidar points were 
considered for the conversion of the CADCD annotations to the KITTI format as part of ground truth.
For each tracklet, the script calculates the truncation, observation angle, pixel coordinates of the 2D bounding box from the given 3D bounding box, and the rotation about the Y-axis of the camera from the given yaw angle about the lidar sensor. 
As the occlusion information was not provided, we set occlusion to ‘0: fully visible’ in the ground truth labels. 
This is to ensure that the annotations can be split into ‘Easy’, ‘Moderate’ and ‘Hard’ categories according to the KITTI standard.

Note that CADCD has 8 cameras and the sample that we worked on consists of 800 frames; 100 from each frame. Our script outputs the annotations for each frame. The output files have names that reflect the camera. E.g CAM 0: 0000000001.txt, whereas CAM 7: 0000000701.txt. The files will be in output directories, e.g annotation_00, annotation_07 etc.


	Example:
	$python filtermoosekitti.py


The figure below shows an example of the annotations before and after running the script. 


![alt text](https://github.com/asvath/cadcd/blob/master/pics/annotations_before_conversion.png)
![alt text](https://github.com/asvath/cadcd/blob/master/pics/annotations_after_conversion.png)
