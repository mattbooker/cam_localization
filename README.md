# cam_localization
cam_localization is a ROS package for creating a single coordinate system across a set of birds-eye view cameras. This project was written as part of my UROP (Undergraduate Research Opportunities Program) at UCI. It was created for use with the set of cameras used in the KCS (Kia Cooperative Systems) Lab.

## Setup
Ensure that the cameras overlap partially, enough for a single marker to be present in both camera views. Also make sure that the cameras are calibrated. Use OpenCV to [calibrate](https://docs.opencv.org/3.1.0/d4/d94/tutorial_camera_calibration.html) the cameras with a set of images. An example set of images can be found under calibration/calib_pics.

Launch the cameras as ROS nodes and place them in namespaces labeled as camera_# replacing # with a number starting from 1. An example file is shown under launch/6_camera_setup.launch.

## Usage
**Calibration** 

Step 1: Place [ARUCO](https://www.uco.es/investiga/grupos/ava/node/26) markers in overlapping regions of the camera views. 

Step 2: Change the number of cameras being used in the launch/calibrate.launch and launch/localize.launch file. Pick which camera you want the origin to be set at.

Step 3: Run the following command

```bash
roslaunch cam_localization calibrate.launch
```

Step 4: Remove the ARUCO markers and run the following command
 
```bash
roslaunch cam_localization localize.launch
```

Any ARUCO marker that moves into the frame of the camera will be tracked and will maintain its coordinates as it moves across different cameras.

![Image](https://github.com/mattbooker/cam_localization/blob/master/result.png)

## Issues
- Currently large changes in height from the calibrated ARUCO markers will cause the system to lose accuracy when moving across camera views

