#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <signal.h>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Empty.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <ardrone_autonomy/distance.h>
#include <ardrone_autonomy/image.h>
#include <ardrone_autonomy/Navdata.h>
#include <geometry_msgs/Twist.h>

using namespace std;

void sigint_handler(int sig) {
	ros::NodeHandle nh;
	ros::Publisher land;
	std_msgs::Empty empty_msg;
	
	land = nh.advertise<std_msgs::Empty>("/ardrone/land", 1, true);
	usleep(2*1000*1000);
	ROS_INFO("INFO: Landing!");
	land.publish(empty_msg);
	
	ros::shutdown();
}

void initialize_drone(ros::NodeHandle n) {
	ros::Publisher takeoff;
	std_msgs::Empty empty_msg;
	
	usleep (2*1000*1000);
	
	ROS_INFO("INFO: Flat trimming...ensure drone is on a flat surface!");
	system("rosservice call /ardrone/flattrim");
	
	takeoff = n.advertise<std_msgs::Empty>("/ardrone/takeoff", 1, true);
	usleep (5*1000*1000);
	ROS_INFO("INFO: Taking Off!");
	takeoff.publish(empty_msg);
}

void set_drone_to_hover(ros::Publisher send_command) {
	geometry_msgs::Twist msg;
	msg.linear.x = 0;
	msg.linear.y = 0;
	msg.linear.z = 0;
	msg.angular.x = 0;
	msg.angular.y = 0;
	msg.angular.z = 0;
	
	send_command.publish(msg);
}

void move_drone(ros::Publisher send_command, float velocity_x, float velocity_z, float yaw_speed) {
	geometry_msgs::Twist msg;
//	msg.linear.x = velocity_x;
	msg.linear.y = 0;
	msg.linear.z = velocity_z;
	msg.angular.x = 1;
	msg.angular.y = 1;
	msg.angular.z = yaw_speed;

	send_command.publish(msg);
}

class increase_altitude {
	int current_altitude;
	int desired_altitude;
	ros::Publisher move_drone_pub;
	ros::NodeHandle nh;
	
	public:
		increase_altitude(ros::Publisher send_command, ros::NodeHandle n, int altitude) {
			current_altitude = 0;
			desired_altitude = altitude;
			move_drone_pub = send_command;
			nh = n;
		}
		
		void callback(const ardrone_autonomy::Navdata::ConstPtr& msg) {
			current_altitude = msg->altd;
			move_drone(move_drone_pub, 0, 0.2, 0);
		}
		
		void run() {
			ros::Subscriber navdata_altitude;
			
			ROS_INFO("INFO: Increasing altitude to %d...", desired_altitude);
			
			navdata_altitude = nh.subscribe<ardrone_autonomy::Navdata> ("/ardrone/navdata", 1, &increase_altitude::callback, this);
			
			while (current_altitude < desired_altitude) {
				ROS_INFO("INFO: Current altitude: %d, Desired altitude: %d", current_altitude, desired_altitude);
				ros::spinOnce();
			}
			set_drone_to_hover(move_drone_pub);
			navdata_altitude.shutdown();
			
			ROS_INFO("INFO: Altitude is now %d", current_altitude);
		}
};

void callback(const ardrone_autonomy::image::ConstPtr& pixels, const ardrone_autonomy::distance::ConstPtr& distance, ros::Publisher send_command) {

	float max_speed = 11.11;
	float veloctiy_cap = 0.1;
	float yaw_cap = 0.2;

	//calculate forward speed
	float velocity = (distance->x - 3)/max_speed;
	velocity = velocity > veloctiy_cap ? veloctiy_cap : velocity;
	velocity = velocity < -veloctiy_cap ? -veloctiy_cap : velocity;
	velocity = 0; //TEMP-REMOVE ONCE IMAGE_PRO HAS BEEN TESTED
	ROS_INFO("Req_velocity: [%f]", velocity);

	//calculate yaw speed
	float yaw_speed = pixels->pixels * (-0.0025);
	yaw_speed = yaw_speed > yaw_cap ? yaw_cap : yaw_speed;
	yaw_speed = yaw_speed < -yaw_cap ? -yaw_cap : yaw_speed;
	ROS_INFO("Yaw Speed: [%f]", yaw_speed);

	if (yaw_speed == 0 && velocity == 0) {
		set_drone_to_hover(send_command);
	} else {
		move_drone(send_command, velocity, 0, yaw_speed);
	}
}

int main (int argc, char **argv) {
	ros::init(argc, argv, "error", ros::init_options::NoSigintHandler);
	signal(SIGINT, sigint_handler);

	ros::NodeHandle n;
	ros::Publisher send_command;

	//connect to the cmd_vel topic
	send_command = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
	
	//start drone
	initialize_drone(n);
	usleep(5*1000*1000);
	set_drone_to_hover(send_command);
	//increase altitude of drone from default
	increase_altitude increase(send_command, n, 1300);
	increase.run();
	
	//subscribe to distance and image_pro
	message_filters::Subscriber<ardrone_autonomy::image> pixel_sub(n, "dpix_pub", 1);
	message_filters::Subscriber<ardrone_autonomy::distance> distance_sub(n, "distance", 1);
	
	//synchronize messages from distance and image_pro
	typedef message_filters::sync_policies::ApproximateTime<ardrone_autonomy::image, ardrone_autonomy::distance> MySyncPolicy;
	message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), pixel_sub, distance_sub);
	
	sync.registerCallback(boost::bind(&callback, _1, _2, send_command));
	
	ros::spin();

	return 0;
}
