//Matthew Booker, UCI, 2018

#include <aruco/aruco.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <map>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <stdlib.h> // getenv
#include <sstream>
#include <string>
#include <image_transport/image_transport.h>
#include <thread>

using namespace std;
using namespace cv;

string window_name = "Camera Localization ";

aruco::CameraParameters cam;
aruco::MarkerDetector marker_detector;
std::vector<aruco::Marker> markers_list;
typedef std::map<int, std::map<int, aruco::Marker>> MarkerMap;
MarkerMap global_marker_map; //map = {marker.id: {camera_id: marker}}

//This function is called everytime a new image is published

void image_cb(const sensor_msgs::ImageConstPtr& original_image, int camera_id)
{
	//Convert ROS-image-msg to a CvImage (OpenCV)
	cv_bridge::CvImagePtr cv_image;
	cv_image = cv_bridge::toCvCopy(original_image, sensor_msgs::image_encodings::BGR8);

	//Call to Aruco to identify markers
	markers_list = marker_detector.detect(cv_image->image, cam, 0.14);	//TODO: size (hard coded)

	float x = 0, y = 0, z = 0;

	for(auto& marker : markers_list) {

		// Add/update the marker in the general_marker_map
		global_marker_map[marker.id][camera_id] = marker;

		aruco::CvDrawingUtils::draw3dAxis(cv_image->image,marker,cam);

		//draw in the image
		marker.draw(cv_image->image);

		if(marker.isPoseValid()) {
			// x = -marker.Tvec.at<Vec3f>(0,0)[0];
			// y = marker.Tvec.at<Vec3f>(0,0)[1];
			// z = marker.Tvec.at<Vec3f>(0,0)[2];
			cout << marker.Tvec << endl;

			x = marker.getCenter().x;
			y = marker.getCenter().y;
		}
	}






	string final_name = window_name + to_string(camera_id);

	string pos = to_string(x) + ", " + to_string(y) + ", " + to_string(z);

	// putText(cv_image->image, pos, Point(100,100), FONT_HERSHEY_SIMPLEX, 1, (255,255,255));
  // cv::line(cv_image->image, Point(0,0), Point(x, y), Scalar(255,255,255));
	//Display the tracked robots
	cv::imshow(final_name, cv_image->image);
	cv::waitKey(1); //Add some delay in miliseconds.

}

void test() {

	//MarkerMap retains last possible values, not most current
	while(ros::ok())
		for(MarkerMap::iterator it = global_marker_map.begin(); it != global_marker_map.end(); it++) {
			// cout << it->first << endl;
		}
}

int main(int argc, char **argv)
{

	string cam_file = "/home/solmaz/Booker/src/cam_localization/calibration/cam.yml";
	cam.readFromXMLFile(cam_file.c_str());

	ros::init(argc, argv, "2cam_localizer");
	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
	image_transport::Subscriber sub = it.subscribe("/camera_1/image_raw", 1, boost::bind(image_cb, _1, 1));	//Webcam 1
	image_transport::Subscriber sub2 = it.subscribe("/camera_2/image_raw", 1, boost::bind(image_cb, _1, 2));	//Webcam 2

	thread first(test);

	ros::spin();

	first.join();

	return 0;
 }
