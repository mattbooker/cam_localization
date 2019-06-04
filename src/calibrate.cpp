// Matthew Booker, UCI, 2018

// OpenCV and Aruco
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <aruco/aruco.h>

//ROS
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>

// Standard Library
#include <iostream>
#include <ros/ros.h>
#include <stdlib.h> // getenv
#include <string>
#include <unordered_map>

#include "CameraGraphCalibrator.hpp"

using namespace cv;

std::string window_name = "Camera Localizer ";
int origin_camera_id;
CameraGraphCalibrator calibrator;

aruco::CameraParameters cam;
aruco::MarkerDetector marker_detector;
std::vector<aruco::Marker> markers_list;

//This function is called everytime a new image is published
void image_cb(const sensor_msgs::ImageConstPtr& original_image, int camera_id)
{
	//Convert ROS-image-msg to a CvImage (OpenCV)
	cv_bridge::CvImagePtr cv_image;
	cv_image = cv_bridge::toCvCopy(original_image, sensor_msgs::image_encodings::BGR8);

	// Undistort the image
	Mat undistorted_image;
	undistort(cv_image->image, undistorted_image, cam.CameraMatrix, cam.Distorsion);

	//Call to Aruco to identify markers
	markers_list = marker_detector.detect(undistorted_image, cam, 0.14);	//0.14m is the large rect

	for(auto& marker : markers_list) {

		if(marker.isPoseValid()) {

			std::vector<Point3f> zero;
			zero.push_back(Point3f(0,0,0));
			zero.push_back(Point3f(0,0,1));
			std::vector<Point2f> marker_pos;

			// Takes the input coordinates (in this case zero) and uses the Rvec, Tvec and camera parameters
			// to project the input coordinates onto the image plane (refer to OpenCV docs for a diagram)
			projectPoints(zero, marker.Rvec, marker.Tvec, cam.CameraMatrix, cam.Distorsion, marker_pos);

			// Add the marker to our calibrator
			calibrator.addVertexInfo(camera_id, marker, marker_pos[0]);
		}
	}

	std::string final_name = window_name + std::to_string(camera_id);


	cv::imshow(final_name, undistorted_image);
	cv::waitKey(1); //Add some delay in miliseconds.

}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "calibrate");
	ros::NodeHandle nh;
	ros::NodeHandle param_nh("~");	// This NodeHandle is for getting parameters

	int number_of_cameras;
	string camera_calibration_file;
	string camera_graph_file;
	param_nh.param<int>("number_of_cameras", number_of_cameras, 0);
	param_nh.param<int>("origin_camera_id", origin_camera_id, 1);
	param_nh.param<string>("camera_calibration_file", camera_calibration_file, " ");
	param_nh.param<string>("camera_graph_file", camera_graph_file, " ");

	calibrator = CameraGraphCalibrator(number_of_cameras, origin_camera_id);
	cam.readFromXMLFile(camera_calibration_file.c_str());

	// Ensure our camera intrinsics size matches our input video resolution
	cam.resize(Size(640,480));

	image_transport::ImageTransport it(nh);

	// Since the number of cameras varies, we need a vector to hold them all (with a size of number_of_cameras)
	std::vector<image_transport::Subscriber> camera_subs(number_of_cameras);

	for(int i = 1; i <= number_of_cameras; i++) {
		std::string current_camera_topic = "/camera_" + std::to_string(i) + "/image_raw";
		camera_subs.push_back(it.subscribe(current_camera_topic, 1, boost::bind(image_cb, _1, i)));

		// Wait to receive a message from each topic to ensure each camera node has started
		ros::topic::waitForMessage<sensor_msgs::Image>(current_camera_topic);
	}

	// Spin once to get the latest set of images
	ros::spinOnce();

	// Need to ensure all cameras are linked to one another
	if(!calibrator.isConnected()) {
		ROS_ERROR("Calibration Error: You do not have markers connecting all cameras.");
	} else {
		calibrator.saveGraph(camera_graph_file.c_str());
		ROS_INFO("Succesfully configured cameras");
	}


	return 0;
 }
