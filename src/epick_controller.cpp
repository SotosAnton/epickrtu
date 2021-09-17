#include <ros/ros.h>
#include <modbus/modbus.h>
#include <epickRTU.h>
#include "std_msgs/String.h"

#define cmd_speed_reg 0x2199
#define eta_speed_reg 0x219B
#define lfrd_reg 0x219A


std::string command;


void commandCallback(const std_msgs::String::ConstPtr& msg){

command = msg->data;

}

int main(int argc, char** argv) {

ros::init(argc, argv, "epick_node");
ros::NodeHandle n;
ros::Subscriber sub = n.subscribe("chatter", 1000, commandCallback);


epick *gripper;

std::string serial_filepath = "/dev/ttyUSB0";

gripper = new epick(serial_filepath);
gripper->connect();


ros::Rate loop_rate(10);

while (ros::ok()){


loop_rate.sleep();
}

gripper->exit();

return 0;

}