#include "Pen.h"
#include "ros/ros.h"
#include "../include/UdpServer.h"
#include <iostream>

int main(int argc, char** argv){
	ros::init(argc, argv, "pen");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("/pen", 1000, penCallback);
	std::string address;
	if(!n.getParam("neato/host", address)){
		ROS_FATAL("Host not set");
		return 1;
	}
	char host[address.length()];
	strcpy(host, address.c_str());

	serv = new UdpServer(host, 31415);
	std::cout << serv->setUp();
	ros::spin();
	return 0;
}

void penCallback(const std_msgs::ByteConstPtr& msg){
	char message[1];
	message[0] = char(msg->data);
	ROS_INFO(message);
	serv->sendMessage(message);
}