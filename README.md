Survey of Motion Tracking Methods Based on Inertial Sensors: A Focus on Upper Limb Human Motion
Alessandro Filippeschi et al. 2017 doi:10.3390/s17061257

Contacts:
- a.filippeschi@sssup.it
- e.ruffaldi@sssup.it

Content
-------

1) data from data capture experiment (see Ethical Approval in paper)
2) code for the analysis

Code
-----------------------------

Use Data: 
- UpS4.mat for the calibration
- UpS10.mat for analysis

Code: starts with MATLAB main.m

Matlab file of IMU+Vicon data
-----------------------------

Vicon and IMU data are Time Aligned

Files
-----------------
Six Sessions with same subject

UpS1.mat
UpS4.mat
UpS8.mat
UpS10.mat
UpS12.mat
UpS20.mat

Content
------------------

Each file contains a struct with:

- imu
	sensors (internal names)
	limbs (names of limbs)
	labels (of data 62 names)

	Various IMUs:
		R_Shoulder Pelvis R_lower_arm L_upper_arm ... 63 values: 1 virtual time, and labels

	Content of Data 

	'timestamp'
	'magx'	'magy'	'magz'	
	'accx'	'accy'	'accz'	
	'acc2x'	'acc2y'	'acc2z'	
	'gyrox'	'gyroy'	'gyroz'	
	'temperature'	
	'mix_temp'	'mix_bat'	
	'joy_volt1'	'joy_volt2'	
	'joy_button_mask1'	'joy_button_mask2'	
	'quat_w'	'quat_x'	'quat_y'	'quat_z'	
	'freq'

- vic
	Various Clusters on Body (11): cam_imu L_lower_arm Trunk R_upper_arm R_Shoulder ... 
	The reference frame is ground fixed

	Content of Data (8 each)
		valid
		position
		quaternion xyzw

Base Poses
----------

These N-Pose and T-Pose are stored inside each sessions at the beginning

	
