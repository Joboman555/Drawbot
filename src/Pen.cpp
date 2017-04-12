#include "Pen.h"
#include "ros/ros.h"
#include "../include/UdpServer.h"

int main(int argc, char** argv){
	ros::init(argc, argv, "pen");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("/pen", 1000, penCallback);
	std::string address;
	if(!n.getParam("host", address)){
		ROS_FATAL("Host not set");
		return 1;
	}
	char *host = new char[address.length() + 1];
	strcpy(host, address.c_str());

	serv = new UdpServer(host, 5000);
	ros::spin();
	return 0;
}

void penCallback(const std_msgs::ByteConstPtr& msg){
	char *message = new char[1];
	message[0] = char(msg->data);

	serv->sendMessage(message);
}