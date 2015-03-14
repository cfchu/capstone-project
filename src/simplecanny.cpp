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

static const char WINDOW[] = "Image window";

class simplecanny
{
	ros::NodeHandle nh_;
	ros::NodeHandle n;
	ros::Publisher pix_pub;
	
	const char* cascade_name;
	cv::CascadeClassifier cascade;
	
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_; //image subscriber
	ros::Publisher distance_pub; //distance publisher
	image_transport::Publisher image_pub_; //image publisher(we subscribe to ardrone image_raw)

	float focal_length; //Variable that holds focal length for distance calibration

	public:
		simplecanny(): it_(nh_){
			image_sub_ = it_.subscribe("/ardrone/image_raw", 1, &simplecanny::imageCb, this);
			image_pub_= it_.advertise("/arcv/Image",1);
			pix_pub= n.advertise<ardrone_autonomy::image>("image_data",1);
			distance_pub = n.advertise<ardrone_autonomy::distance>("distance", 1000);
			cv::namedWindow(WINDOW);
			
			/* Create cascade classifier, loaded with XML file */
			cascade_name = "/home/odroid/catkin_ws/src/ardrone_autonomy/XML/lbpcascade_frontalface.xml";
			cascade.load(cascade_name);
			
			focal_length = 0;
		}

		~simplecanny(){
			cv::destroyWindow(WINDOW);
		}

		void imageCb(const sensor_msgs::ImageConstPtr& msg){
			/* OpenCV image pointer, copy message and point to copied image */
			cv_bridge::CvImagePtr cv_imgptr_orig;
			cv::Mat cv_imgptr;
			cv_imgptr_orig = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
			cv::resize(cv_imgptr_orig->image, cv_imgptr, Size(), 1.25, 1.25, INTER_AREA);

			/* Create new CV image w/ grayscale */
			cv::Size image_size(cv_imgptr.cols, cv_imgptr.rows);
			cv::Mat gray = cv::Mat(image_size, CV_8UC1);
			cv::cvtColor(cv_imgptr, gray, CV_BGR2GRAY);

			/*Create vector of type Rect and then use cascade classifier to detect all rects and store in faces */
			vector<cv::Rect> faces;
			faces.clear();
			cascade.detectMultiScale(gray, faces, 1.2, 3, 0, cv::Size( 40, 40));

			vector<int> face_area;
			ardrone_autonomy::image msg_pub;
			msg_pub.yaw = 0;
			msg_pub.height = 0;

			/* Go through each face*/
			for (int i = 0; i < faces.size(); i++){
				CvPoint ul; CvPoint lr;
				ul.x = faces[i].x; ul.y = faces[i].y;
				lr.x = faces[i].x + faces[i].width; lr.y = faces[i].y + faces[i].height;
				face_area.push_back((lr.x-ul.x)*(ul.y-lr.y));
			}

			static cv::Scalar RED = cv::Scalar(0, 0, 255);
			int max = 0;
			int index = 0;

			for (int i = 0; i < face_area.size(); ++i){
				if (face_area[i] > max)
					index = i;
			}

			/* Iterate through all faces found again, */
			if (faces.size()>0){
				CvPoint ul_final;
				CvPoint lr_final;

				ul_final.x = faces[index].x;
				ul_final.y = faces[index].y;

				lr_final.x = ul_final.x + faces[index].width;
				lr_final.y = ul_final.y + faces[index].height;

				//--------------------- calculating distance here---------------------------

				//check if focal length is 0 and initialize it. the facewidth is approximately 6 inches and distance is 2 m.
				if (focal_length == 0)
					focal_length = 200 * faces[index].width / 15.24;

				//use focal length to calculate distance away.
				msg_pub.distance = focal_length * 15.24 / faces[index].width;
//				ardrone_autonomy::distance distance_msg;
//				distance_msg.header.stamp = ros::Time::now();
//				distance_msg.x = distance;
//				distance_pub.publish(distance_msg);

				//--------------------- calculating distance ends here---------------------------
				
				msg_pub.height = (((lr_final.y+ul_final.y)/2) - ((cv_imgptr.rows)/2));

				cv::rectangle(cv_imgptr, ul_final, lr_final, RED, 3, 8, 0);

				msg_pub.header.stamp = ros::Time::now();
				msg_pub.yaw = (((lr_final.x+ul_final.x)/2) - ((cv_imgptr.cols)/2));
				pix_pub.publish(msg_pub);
			}
			cv::imshow(WINDOW,cv_imgptr);
			cv::waitKey(2);
		}
};

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
			
			while (is_ready == false)
				ros::spinOnce();
			
			ready_sub.shutdown();
			
			return;
		}
};


int main(int argc, char** argv)
{
	ros::init(argc, argv, "image_processing");
	
	//wait for ready signal from error module
	ready obj;
	obj.run();
	
	simplecanny processing_obj;
		
	ros::spin();

	return 0;
}
