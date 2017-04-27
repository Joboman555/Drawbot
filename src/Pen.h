#ifndef PEN_H
#define PEN_H
#include "std_msgs/Byte.h"
#include "../include/UdpServer.h"

UdpServer *serv;

void penCallback(const std_msgs::ByteConstPtr& msg);

#endif //PEN_H