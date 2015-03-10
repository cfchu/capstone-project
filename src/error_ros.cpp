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
	msg.linear.x = velocity_x;
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
			move_drone(move_drone_pub, 0, 0.4, 0);
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

class PI_controller {
	double time_stamp_prev;
	float integral;
	float SP;
	float kp, ki, kd;
	float outMax, outMin;
	float prev_error[3];
	float output_slew_rate, prev_output;
	
	public:
		PI_controller(float SP_in, float slew_rate_in, float outMax_in, float outMin_in, float kp_in, float ki_in, float kd_in) {
			time_stamp_prev = 0;
			integral = 0;
			kp = kp_in; ki = ki_in; kd = kd_in;
			SP = SP_in;
			outMax = outMax_in;
			outMin = outMin_in;
			prev_error[0] = 0; prev_error[1] = 0; prev_error[2] = 0;
			output_slew_rate = slew_rate_in;
			prev_output = 0;
		}
		
		float compute_output(float PV){
			float error, dt, output;
			
			double time_stamp_curr = ros::Time::now().toSec();
			dt = time_stamp_curr - time_stamp_prev;
			//compute error
			error = SP - PV;
			
			//compute integral term
			integral = integral + (error*dt);
			
			if (integral > outMax) integral = outMax;
			else if(integral < outMin) integral = outMin;
			
			//compute derivative term
			float derivative = ((error + 3*prev_error[2] - 3*prev_error[1] - prev_error[0])/6)/dt;
			
			ROS_INFO("Error: [%f] , Integral: [%f], Derivative: [%f]", error, integral, derivative);
//			ROS_INFO("Current: [%f], Prev: [%f]", time_stamp_curr, time_stamp_prev);
			ROS_INFO("dt: [%f]", dt);
			ROS_INFO("prev_error[0]: [%f], prev_error[1]: [%f], prev_error[2]: [%f]", prev_error[0], prev_error[1], prev_error[2]);
			
			//compute output
			output = (kp*error) + (ki*integral) + (kd*derivative);
			//slew rate
			if (output-prev_output > output_slew_rate) output = prev_output+output_slew_rate;
			else if (output-prev_output < -output_slew_rate) output = prev_output-output_slew_rate;
			//cap output
			if (output > outMax) output = outMax;
			else if(output < outMin) output = outMin;
			
			time_stamp_prev = time_stamp_curr;
			prev_output = output;
			prev_error[1] = prev_error[0];
			prev_error[2] = prev_error[1];
			prev_error[0] = error;
			
			return output;
		}
};

void callback(const ardrone_autonomy::image::ConstPtr& pixels, \
			  const ardrone_autonomy::distance::ConstPtr& distance, \
			  ros::Publisher send_command, \
			  PI_controller &velocity_PI, \
			  PI_controller &yaw_PI) {

	//calculate forward speed
	float velocity = -velocity_PI.compute_output(distance->x);
	ROS_INFO("Distance: [%f] , ----->Req_velocity: [%f]<-----\n", distance->x, velocity);

	//calculate yaw speed
	float yaw_speed = yaw_PI.compute_output(pixels->pixels);
	ROS_INFO("----->Yaw Speed: [%f]<------\n", yaw_speed);

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
	
	//intialize PI_controller objects
	PI_controller velocity_PI(200, 0.1, 0.1, -0.1, 0.001, 0, 0.0015);
	PI_controller yaw_PI     (0, 0.1, 0.5, -0.5, 0.0015, 0, 0.004);

	//connect to the cmd_vel topic
	send_command = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
	
	//start drone
	initialize_drone(n);
	usleep(3*1000*1000);
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
	
	sync.registerCallback(boost::bind(&callback, _1, _2, send_command, velocity_PI, yaw_PI));
	
	ros::spin();

	return 0;
}
