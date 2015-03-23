//Capstone Project: Autonomous Action-Sport Camera Drone
//Image Processing Module
//Purpose: Runs facial recognition of video feed from drone. Then calculates the deviation of the face
//		   from the centre of the image & the distance from the camera to the user.
//	Input: Video from drone
//	Outputs: Vertical & Horizontal deviation and distance (cm)

#include <stdio.h>
#include <iostream>
#include <cmath>
#include <vector>
#include <math.h>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>

#include <ardrone_autonomy/image.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <ardrone_autonomy/distance.h>
#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <opencv/highgui.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <cv_bridge/cv_bridge.h>

using namespace std;
using namespace cv;

////////////////////////////////////////////////////////////////////////////////
// class facial_detection
////////////////////////////////////////////////////////////////////////////////

class facial_detection
{
	ros::NodeHandle nh_;
	ros::NodeHandle n;
	ros::Publisher pix_pub;
	
	const char* cascade_name;
	CascadeClassifier cascade;
	
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_; 				//image subscriber
	ros::Publisher distance_pub; 							//distance publisher
	image_transport::Publisher image_pub_; 					//image publisher (we subscribe to ardrone image_raw)

	float focal_length; 									//Variable that holds focal length for distance calibration

	public:
		facial_detection(): it_(nh_){
			image_sub_ = it_.subscribe("/ardrone/image_raw", 1, &facial_detection::imageCb, this);
			image_pub_ = it_.advertise("/arcv/Image",1);
			
			//advertise topics to publish pixel deviation and distance 
			pix_pub = n.advertise<ardrone_autonomy::image>("image_data",1);
			distance_pub = n.advertise<ardrone_autonomy::distance>("distance", 1000);

			
			/* Create cascade classifier, loaded with face XML file*/
			cascade_name = "/home/odroid/catkin_ws/src/ardrone_autonomy/XML/lbpcascade_frontalface.xml";
			cascade.load(cascade_name);
			
			namedWindow("image");
			
			focal_length = 0;
		}

		~facial_detection(){
			destroyWindow("image");
		}

		void imageCb(const sensor_msgs::ImageConstPtr& msg){
			cv_bridge::CvImagePtr cv_imgptr_orig;
			Mat cv_imgptr;
			vector<int> face_area;
			int max = 0;
			int index = 0;
			
			//Initialise message to publish data to error module
			ardrone_autonomy::image msg_pub;
			msg_pub.yaw = 0;
			msg_pub.height = 0;
			msg_pub.distance = 250;
			
			//OpenCV image pointer, copy message, enlarge and point to copied image
			cv_imgptr_orig = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
			resize(cv_imgptr_orig->image, cv_imgptr, Size(), 1.25, 1.25, INTER_AREA);

			//Create new CV image w/ grayscale
			Size image_size(cv_imgptr.cols, cv_imgptr.rows);
			Mat gray = Mat(image_size, CV_8UC1);
			cvtColor(cv_imgptr, gray, CV_BGR2GRAY);

			//Create vector of type Rect and then use cascade classifier to store all detected faces
			vector<Rect> faces;
			faces.clear();
			cascade.detectMultiScale(gray, faces, 1.2, 3, 0, Size( 40, 40));

			//Colour of rectangle that will be drawn around detected face
			Scalar RED = Scalar(0, 0, 255);
			
			//Get pixel coordinates for upper left and lower right corner of detected faces
			for (int i = 0; i < faces.size(); i++){
				CvPoint ul; 
				CvPoint lr;
				
				ul.x = faces[i].x; 
				ul.y = faces[i].y;
				lr.x = faces[i].x + faces[i].width; 
				lr.y = faces[i].y + faces[i].height;
				
				face_area.push_back((lr.x-ul.x)*(ul.y-lr.y));
			}

			//Iterate thro detected faces and pick the largest face
			for (int i = 0; i < face_area.size(); i++){
				if (face_area[i] > max)
					index = i;
					max = face_area[i];
			}

			//Calculate distance & pixel deviation of selected face & publish this info
			if (faces.size()>0){
				CvPoint ul_final;
				CvPoint lr_final;

				ul_final.x = faces[index].x;
				ul_final.y = faces[index].y;

				lr_final.x = ul_final.x + faces[index].width;
				lr_final.y = ul_final.y + faces[index].height;

				//calculating distance
				//check if focal length is 0 and initialize it. the face width is approximately 15.24cm and distance is 250cm.
				if (focal_length == 0)
					focal_length = 250 * faces[index].width / 15.24;

				msg_pub.distance = focal_length * 15.24 / faces[index].width;

				//calculate vertical and horizontal pixel deviation				
				msg_pub.height = (((lr_final.y+ul_final.y)/2) - ((cv_imgptr.rows)/2));
				msg_pub.yaw = (((lr_final.x+ul_final.x)/2) - ((cv_imgptr.cols)/2));
							
				rectangle(cv_imgptr, ul_final, lr_final, RED, 3, 8, 0);

				msg_pub.header.stamp = ros::Time::now();
				pix_pub.publish(msg_pub);
			}
			
			imshow("image",cv_imgptr);
			waitKey(2);
		}
};

////////////////////////////////////////////////////////////////////////////////
// class ready
////////////////////////////////////////////////////////////////////////////////

class ready {
	int is_ready;
	ros::NodeHandle nh;
	
	public:
		ready() {
			is_ready = 0;
		}
		
		void callback(const std_msgs::Bool::ConstPtr& msg) {
			is_ready = msg->data;
		}
		
		void run() {
			ros::Subscriber ready_sub;
			
			ready_sub = nh.subscribe<std_msgs::Bool> ("ready", 1, &ready::callback, this);
			
			//loop until ready signal is received
			while (is_ready == false)
				ros::spinOnce();
			
			ready_sub.shutdown();
			
			return;
		}
};

////////////////////////////////////////////////////////////////////////////////
// main
////////////////////////////////////////////////////////////////////////////////

int main(int argc, char** argv)
{
	ros::init(argc, argv, "image_processing");
	
	//wait for ready signal from error module
	ready obj;
	obj.run();
	
	facial_detection proc_obj;
		
	ros::spin();

	return 0;
}
