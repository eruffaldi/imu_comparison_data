

This is the dataset and the data associated to the Practical Review paper "Survey of Motion Tracking Methods Based on Inertial Sensors: A Focus on Upper Limb Human Motion" published on Sensors in 2017 [doi:10.3390/s17061257](http://www.mdpi.com/1424-8220/17/6/1257)

Contacts:
- [Alessandro Filippeschi](https://github.com/alessandrofilippeschi)
- [Emanuele Ruffaldi](https://github.com/eruffaldi)

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

# References

Please cite the following paper if you use the data or code:

	Filippeschi, A.; Schmitz, N.; Miezal, M.; Bleser, G.; Ruffaldi, E.; Stricker, D.	
	"Survey of Motion Tracking Methods Based on  Inertial Sensors: A Focus on Upper Limb Human Motion".
	Sensors 2017, 17, 1257.

The UKF method discussed in the review is from the same authors:

	Peppoloni, Lorenzo, et al. 
	"A novel 7 degrees of freedom model for upper limb kinematic reconstruction based on wearable sensors".
	Intelligent Systems and Informatics (SISY), IEEE, 2013.

 
