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
#include <map>

using namespace cv;

std::string window_name = "Camera Localizer ";

aruco::CameraParameters cam;
aruco::MarkerDetector marker_detector;
std::vector<aruco::Marker> markers_list;
typedef std::map<int, std::map<int, aruco::Marker>> MarkerMap;
MarkerMap global_marker_map; //MarkerMap = {marker.id: {camera_id: marker}}
std::map<int, Point2f> anchor_pos; // {camera_id, Point}

Point2f adjustPos(int camera_id, Point2f pos_in_camera) {
	if(camera_id == 1)
		return pos_in_camera;

	Point2f	difference_from_anchor = anchor_pos[1] - pos_in_camera;

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

	float x = 0, y = 0, z = 0;

	for(auto& marker : markers_list) {

		// Add/update the marker in the general_marker_map
		global_marker_map[marker.id][camera_id] = marker;

		// aruco::CvDrawingUtils::draw3dAxis(cv_image->image,marker,cam);

		//draw in the image
		// marker.draw(cv_image->image);

		if(marker.isPoseValid()) {
			x = marker.Tvec.at<Vec3f>(0,0)[0];
			y = marker.Tvec.at<Vec3f>(0,0)[1];
			z = marker.Tvec.at<Vec3f>(0,0)[2];
			// std::cout << "x: " << x << ", y: " << y << ", z: " << z << std::endl;

			std::vector<Point3f> axisPoints;
			std::vector<Point2f> origin_of_marker;

			axisPoints.push_back(Point3f(0, 0, 0));

			projectPoints(axisPoints, marker.Rvec, marker.Tvec, cam.CameraMatrix, cam.Distorsion, origin_of_marker);

			anchor_pos[camera_id] = origin_of_marker[0];

			Point2f relative_pos = adjustPos(camera_id, origin_of_marker[0]);

			std::string pos = std::to_string(relative_pos.x) + ", " + std::to_string(relative_pos.y);

			putText(cv_image->image, pos, origin_of_marker[0],  FONT_HERSHEY_PLAIN, 1, (255,255,255));
			std::cout << "camera: " << camera_id << ". x: " << origin_of_marker[0].x << ", y: " << origin_of_marker[0].y << std::endl;
		}
	}






	std::string final_name = window_name + std::to_string(camera_id);


  // cv::line(cv_image->image, Point(0,0), Point(x, y), Scalar(255,255,255));
	cv::imshow(final_name, cv_image->image);
	cv::waitKey(1); //Add some delay in miliseconds.

}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "localizer");
	ros::NodeHandle nh;
	ros::NodeHandle param_nh("~");	// This NodeHandle is for getting parameters

	int number_of_cameras;
	param_nh.param<int>("number_of_cameras", number_of_cameras, 0);

	std::string cam_file = "/home/solmaz/Booker/src/cam_localization/calibration/cam.yml";
	cam.readFromXMLFile(cam_file.c_str());

	image_transport::ImageTransport it(nh);

	// Since the number of cameras varies, we need a vector to hold them all
	std::vector<image_transport::Subscriber> camera_subs(number_of_cameras); // Create a vector with a size of number_of_cameras

	for(int i = 1; i <= number_of_cameras; i++) {
		camera_subs.push_back(it.subscribe("/camera_" + std::to_string(i) + "/image_raw", 1, boost::bind(image_cb, _1, i)));
	}


	ros::spin();

	return 0;
 }