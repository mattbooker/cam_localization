//Matthew Booker, UCI, 2018

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

#include "CameraGraph.hpp"

using namespace cv;

std::string window_name = "Camera Localizer ";

aruco::CameraParameters cam;
aruco::MarkerDetector marker_detector;
std::vector<aruco::Marker> markers_list;

CameraGraph camGraph;

//This function is called everytime a new image is published
void image_cb(const sensor_msgs::ImageConstPtr& original_image, int camera_id)
{
	//Convert ROS-image-msg to a CvImage (OpenCV)
	cv_bridge::CvImagePtr cv_image;
	cv_image = cv_bridge::toCvCopy(original_image, sensor_msgs::image_encodings::BGR8);

	Mat undistorted_image;
	undistort(cv_image->image, undistorted_image, cam.CameraMatrix, cam.Distorsion);

	//Call to Aruco to identify markers
	markers_list = marker_detector.detect(undistorted_image, cam, 0.14);	//0.14m is the large rect

	float x = 0, y = 0, z = 0;

	for(auto& marker : markers_list) {

		aruco::CvDrawingUtils::draw3dAxis(undistorted_image,marker,cam);

		//draw in the image
		marker.draw(undistorted_image);

		if(marker.isPoseValid()) {

			std::vector<Point3f> zero;
			zero.push_back(Point3f(0, 0, 0));
			std::vector<Point2f> origin_of_marker;

			projectPoints(zero, marker.Rvec, marker.Tvec, cam.CameraMatrix, cam.Distorsion, origin_of_marker);

			Point2f global_pos = camGraph.getGlobalPos(camera_id, origin_of_marker[0]);

			std::string pos = std::to_string(int(global_pos.x)) + ", " + std::to_string(int(global_pos.y));

			putText(undistorted_image, pos, origin_of_marker[0] - Point2f(50, 50),  FONT_HERSHEY_PLAIN, 2, Scalar(0,0,255));
		}
	}

	std::string final_name = window_name + std::to_string(camera_id);

	cv::imshow(final_name, undistorted_image);
	cv::waitKey(1); //Add some delay in miliseconds.

}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "localizer");
	ros::NodeHandle nh;
	ros::NodeHandle param_nh("~");	// This NodeHandle is for getting parameters

	int number_of_cameras;
	int origin_camera_id;
	string camera_calibration_file;
	string camera_graph_file;
	param_nh.param<int>("number_of_cameras", number_of_cameras, 0);
	param_nh.param<int>("origin_camera_id", origin_camera_id, 0);
	param_nh.param<string>("camera_calibration_file", camera_calibration_file, " ");
	param_nh.param<string>("camera_graph_file", camera_graph_file, " ");

	cam.readFromXMLFile(camera_calibration_file.c_str());

	// Ensure our camera intrinsics size matches our input video resolution
	cam.resize(Size(640,480));

	camGraph = CameraGraph(number_of_cameras, origin_camera_id);
	camGraph.loadGraph(camera_graph_file.c_str());

	image_transport::ImageTransport it(nh);

	// Since the number of cameras varies, we need a vector to hold them all
	std::vector<image_transport::Subscriber> camera_subs(number_of_cameras); // Create a vector with a size of number_of_cameras

	for(int i = 1; i <= number_of_cameras; i++) {
		camera_subs.push_back(it.subscribe("/camera_" + std::to_string(i) + "/image_raw", 1, boost::bind(image_cb, _1, i)));
	}


	ros::spin();

	return 0;
 }
