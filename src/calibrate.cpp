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

#include "CameraGraphCalibrator.hpp"

using namespace cv;

std::string window_name = "Camera Localizer ";
int origin_camera_id;
CameraGraphCalibrator calibrator;

aruco::CameraParameters cam;
aruco::MarkerDetector marker_detector;
std::vector<aruco::Marker> markers_list;
std::unordered_map<int, Point2f> anchor_pos; // {camera_id, Point}

Point2f adjustPos(int camera_id, Point2f pos_in_camera) {
	if(camera_id == origin_camera_id)
		return pos_in_camera;

	Point2f	difference_from_anchor = anchor_pos[origin_camera_id] - pos_in_camera;

	return pos_in_camera + difference_from_anchor;

}

//This function is called everytime a new image is published
void image_cb(const sensor_msgs::ImageConstPtr& original_image, int camera_id)
{
	//Convert ROS-image-msg to a CvImage (OpenCV)
	cv_bridge::CvImagePtr cv_image;
	cv_image = cv_bridge::toCvCopy(original_image, sensor_msgs::image_encodings::BGR8);

	//Call to Aruco to identify markers
	markers_list = marker_detector.detect(cv_image->image, cam, 0.14);	//0.14m is the large rect

	for(auto& marker : markers_list) {

		if(marker.isPoseValid()) {


			std::vector<Point3f> zero;
			zero.push_back(Point3f(0,0,0));
			std::vector<Point2f> marker_pos;

			// Takes the input coordinates (in this case zero) and uses the Rvec, Tvec and camera parameters to project the input
			// coordinates onto the image plane (refer to OpenCV docs for a diagram)
			projectPoints(zero, marker.Rvec, marker.Tvec, cam.CameraMatrix, cam.Distorsion, marker_pos);

			calibrator.addVertexInfo(camera_id, marker, marker_pos[0]);

			// Save the coordinates of the marker
			anchor_pos[camera_id] = marker_pos[0];

			// Get the position w.r.t the origin camera
			// Point2f global_pos = adjustPos(camera_id, marker_pos[0]);

			Point2f global_pos = calibrator.getGlobalPos(camera_id, marker_pos[0]);

			std::string pos = std::to_string(global_pos.x) + ", " + std::to_string(global_pos.y);

			putText(cv_image->image, pos, marker_pos[0],  FONT_HERSHEY_PLAIN, 1, (255,255,255));
		}
	}






	std::string final_name = window_name + std::to_string(camera_id);


  // cv::line(cv_image->image, Point(0,0), Point(x, y), Scalar(255,255,255));
	cv::imshow(final_name, cv_image->image);
	cv::waitKey(1); //Add some delay in miliseconds.

}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "calibrate");
	ros::NodeHandle nh;
	ros::NodeHandle param_nh("~");	// This NodeHandle is for getting parameters

	int number_of_cameras;
	param_nh.param<int>("number_of_cameras", number_of_cameras, 0);
	param_nh.param<int>("origin_camera_id", origin_camera_id, 2);

	calibrator = CameraGraphCalibrator(number_of_cameras, origin_camera_id);

	std::string cam_file = "/home/solmaz/Booker/src/cam_localization/calibration/cam.yml";
	cam.readFromXMLFile(cam_file.c_str());

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
		calibrator.saveGraph("/home/solmaz/Booker/src/cam_localization/calibration/camera_graph.txt");
	}


	ros::spin();



	return 0;
 }
