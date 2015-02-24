#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>
#include <ardrone_autonomy/distance.h>

using namespace std;

int main (int argc, char **argv) {
	
	ros::init(argc, argv, "distance");
	ros::NodeHandle n;
	
	ros::Publisher chatter_pub = n.advertise<ardrone_autonomy::distance>("distance", 1000);
	
	FILE* in;
	int signal_level;
	int iterations = 100;
	int index;
	char buff[4];
	float average_level;
	float signal_dbm, distance;
	float signal_strength_array[iterations], exp;
	float sum;
	float k;

	if (!(in = popen("iwconfig wlan0 | grep Signal | cut -d'=' -f3 | cut -d' ' -f1", "r"))) {
		return 1;
        }

	while(fgets(buff, sizeof(buff), in)!=NULL){
		if (atoi(buff) != 0)
			signal_level = atoi(buff);
	}
	pclose(in);

//	k = -20*log10(3) + ( (signal_level/2) - 100);
	k = 3 + signal_level;

	for (int i=0; i<iterations; i++)
		signal_strength_array[i]=signal_level;


	index = 0;
	setvbuf(in,NULL,_IONBF,0);														//set buffer size to zero for stdout

	while (ros::ok()) {

		usleep(50000);

		if (!(in = popen("iwconfig wlan0 | grep Signal | cut -d'=' -f3 | cut -d' ' -f1", "r"))) {
			return 1;
		}

		while(fgets(buff, sizeof(buff), in)!=NULL){
                        if (atoi(buff) != 0)
        			signal_level = atoi(buff);
    		}
		pclose(in);

		signal_strength_array[index] = signal_level;
		index++;

		if (index >= iterations)
			index = 0;

		sum = 0;

		for (int i = 0; i<iterations ; i++)
			sum  += signal_strength_array[i];

		average_level = sum/iterations;

//      exp = (k - (signal_dbm + 67.6476)) / 20;
//		exp = (k + signal_dbm)/(-20);
//      distance = pow(10,exp);

		distance = (k - average_level);
		
		//Publish distance
		ardrone_autonomy::distance msg;
		stringstream ss;
		
		ss << "Distance: "<<distance<<endl<<endl;
		msg.header.frame_id= "Distance";
		msg.header.stamp = ros::Time::now();
		msg.x = distance;

		chatter_pub.publish(msg);
		
		ROS_INFO("%f", msg.x);

//		cout<<average_level<<endl;
//		cout<<k<<endl;
//		cout<<distance<<endl;
	}

	return 0;
}
