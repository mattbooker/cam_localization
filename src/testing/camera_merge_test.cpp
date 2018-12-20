#include <aruco/aruco.h>
#include <aruco/cameraparameters.h>
#include <aruco/cvdrawingutils.h>
#include <aruco/markerdetector.h>
#include <cv_bridge/cv_bridge.h>
#include <fstream>
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

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "coop_localization/rel.h"
#include "name_map.h" //List that contains the information for every marker

using namespace std;
using namespace cv;

//--------------Size is changed from 0.17 to 0.14------------------
float x_pxl, y_pxl, size=0.14;
const double pi = 3.14159265;
string home, yml_file;
ros::Publisher rel_meas_pub;

map<string,coop_localization::rel> rel;
string names[4] = {"inky", "pinky", "clyde", "blinky"};
int i = 0;
map<string,bool> measurement;
// ROS
image_transport::Publisher pub;
// ArUco
aruco::MarkerDetector MDetector;
vector<aruco::Marker> Markers;
aruco::CameraParameters CamParam;
// OpenCV
namespace enc = sensor_msgs::image_encodings;
static const char WINDOW[] = "Multiagent Localization Camera";// For the OpenCV-window.


string getEnvVar(string const& key)
{
	char const* val = getenv(key.c_str());
	return val == NULL ? string() : string(val);
}

//This function is called everytime a new image is published
void image_cb(const sensor_msgs::ImageConstPtr& cam_1, const sensor_msgs::ImageConstPtr& cam_2)
{
	//Convert ROS-image-msg to a CvImage (OpenCV)
	cv_bridge::CvImagePtr cv_cam1 = cv_bridge::toCvCopy(cam_1, enc::BGR8);
	cv_bridge::CvImagePtr cv_cam2 = cv_bridge::toCvCopy(cam_2, enc::BGR8);

	cv::Mat combine;
  vconcat(cv_cam1->image, cv_cam2->image,combine);

	//Call to Aruco to identify markers
	MDetector.detect(combine, Markers,CamParam, size);

	// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	// ~~~~~~~~~~~~~~~~START LOOP for the markers~~~~~~~~~~~~~~~~~~~~~

	//Variables to describe robot's position
	float x, y, z;
	float roll,yaw,pitch;
	float real_angle;

	for (unsigned int i=0;i<Markers.size();i++) { // Runs through every detected marker

		int id = Markers[i].id;
		// Checks if the detected marker is defined in the name_map
		//If the specified id is not in the name_map than, map::end is returned (past-the-end element in the container)
		map<int,boost::tuple<string,int,string> >::iterator j=name_map.find(id);

		// aruco::CvDrawingUtils::draw3dAxis(combine,Markers[i],CamParam);

		//get the center of the marker
		x_pxl = Markers[i].getCenter().x;
		y_pxl = Markers[i].getCenter().y;

		//skip if the marker is not defined
		if ((j != name_map.end()))
		{
			//get the size of the marker
			string size_m = j->second.get<2>();

			if (size_m == "l")
			{
				//get name of the marker
				string 	name = j->second.get<0>();

				// Calculate the detected pose
				// According to https://github.com/warp1337/ros_aruco
				x =  Markers[i].Tvec.at<Vec3f>(0,0)[2];//- and 0
				y = -Markers[i].Tvec.at<Vec3f>(0,0)[0];//+ and 1
				z = -Markers[i].Tvec.at<Vec3f>(0,0)[1];//+ and 2
				cv::Mat rot_mat(3,3,cv::DataType<float>::type);
				cv::Rodrigues(Markers[i].Rvec,rot_mat);

				roll = acos(rot_mat.at<float>(2,2)) - CV_PI/2;
				pitch = atan2(rot_mat.at<float>(0,2), rot_mat.at<float>(1,2)) + CV_PI/2;
				//yaw = atan2(rot_mat.at<float>(2,0), rot_mat.at<float>(2,1)) - CV_PI;
				yaw = atan2(rot_mat.at<float>(1,0), rot_mat.at<float>(0,0));
				yaw += CV_PI/2;

				//--------------------------The real angle that is used (from 0 to 2pi)---------------------------

				real_angle = -1 * (atan2(rot_mat.at<float>(1,0), rot_mat.at<float>(0,0)) - CV_PI / 2);

				if (real_angle < 0)
					real_angle += (CV_PI * 2);

				//------------------------------------------------------------------------------------------------
				double angle_in_deg = real_angle * 180 / CV_PI;

				ROS_INFO_THROTTLE(2, "Robot: %s, Real Angle: %f", name.c_str(), angle_in_deg);

				geometry_msgs::Twist twist_msg;

				twist_msg.linear.x = -y;
				twist_msg.linear.y = z;
				twist_msg.angular.z = real_angle;

				rel[name].current_time = ros::Time::now();
				rel[name].meas = twist_msg;
				rel[name].agent_a = "abs";
				rel[name].agent_b = name;
				measurement[name] = true;

				//draw a text string
				cv::putText(combine,name,cv::Point(x_pxl-50,y_pxl-30),2,1, cv::Scalar(0,255,0));
				//draw the marker in the input image
				Markers[i].draw(combine,cv::Scalar(0,255,0),2);
			}
		}
		else{
			//draw a text string
			cv::putText(combine,"Skipped!",cv::Point(x_pxl-50,y_pxl-30),2,1, cv::Scalar(0,0,255));
			//draaw a marker in the input image
			Markers[i].draw(combine,cv::Scalar(0,0,255),2);
		}
	}

	//Display the tracked robots

  cv::imshow(WINDOW, combine);
	// cv::imshow(WINDOW2, cv_ptr_2->image);
	cv::waitKey(1); //Add some delay in miliseconds.
}

int main(int argc, char **argv)
{

	ros::init(argc, argv, "cam_merge");
	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
	rel_meas_pub = nh.advertise< coop_localization::rel>("/central_processing/rel_meas", 1,false);
	// image_transport::Subscriber sub = it.subscribe("/camera_1/image_raw", 1, image_cb);	//Webcam 2
	// image_transport::Subscriber sub2 = it.subscribe("/camera_2/image_raw", 1, image_cb);	//Webcam 1

  message_filters::Subscriber<sensor_msgs::Image> cam_1(nh, "/camera_1/image_raw", 1);
  message_filters::Subscriber<sensor_msgs::Image> cam_2(nh, "/camera_2/image_raw", 1);
  // message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image> sync(cam_1, cam_2, 10);

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> approx_time;
  message_filters::Synchronizer<approx_time> sync(approx_time(10), cam_1, cam_2);
  sync.registerCallback(boost::bind(&image_cb, _2, _1));


	home = getEnvVar(string("HOME"));
	yml_file = home + string("/ros/src/coop_localization/calibration/new_cal.yml");
	// Read in the Kinect calibration file
	CamParam.readFromXMLFile(yml_file.c_str());

  	cv::namedWindow(WINDOW, CV_WINDOW_AUTOSIZE);
	MDetector.setThresholdParams(7,7);
	cv::destroyWindow(WINDOW);

	// was 5
	ros::Rate loop_rate(10); //16 Hz and four robots -> individual robot is published with 4 Hz

	while (ros::ok())
	{
		ros::spinOnce();
		if (i==4)
			i=0;

		//check if the detected marker satisfies all the requirements
		if (measurement[names[i]]==true)
		{
			rel_meas_pub.publish(rel[names[i]]);
			measurement[names[i]] = false;
		}

		i++;
		loop_rate.sleep();
	}
 }
