/*
 * HNSDeviceController.cpp
 *
 *  Created on: Apr 17, 2020
 *      Author: morin
 */

#include <hns_device_controller/HNSDeviceController.h>
using namespace hns_device_controller;
using namespace hns_msgs;

ros::Publisher HNSDeviceController::m_cmd_pub;

HNSDeviceController::HNSDeviceController(){
	ros::NodeHandle nh;
	m_cmd_pub = nh.advertise<hns_msgs::HNSCommand>("/cmd_HNS", 1);
	m_input_sub = nh.subscribe<hns_msgs::HNSCommand>("/user_input", 1, &HNSDeviceController::setCurrentCMD, this);
	m_timer = nh.createTimer(ros::Duration(0.1), &HNSDeviceController::timerCallback, this);
	signal(SIGINT, HNSDeviceController::sigIntHandler);
	m_isInit = false;
}

HNSDeviceController::~HNSDeviceController(){
}

void HNSDeviceController::setCurrentCMD(const HNSCommand msg){
	if (msg.valve_num < HNSCommand::RESET || msg.valve_num > HNSCommand::VALVE_4 ||
			msg.option_num < 0 || msg.option_num > 10){
		ROS_ERROR("HNSDeviceController: setCurrentCMD: received topic msg is not in proper form");
		return;
	}
	m_current_cmd = msg;
	m_isInit = true;
	return;
}

void HNSDeviceController::timerCallback(const ros::TimerEvent& event){
	if (!m_isInit){
		ROS_WARN("HNSDeviceController: timerCallback: current_cmd is not set yet");
		return;
	}
	m_cmd_pub.publish(m_current_cmd);
	return;
}

void HNSDeviceController::sigIntHandler(int sig){
	ROS_INFO("signal Interrupt detected: quit after 3 seconds..");
	HNSCommand cmd;
	cmd.valve_num = HNSCommand::RESET;
	ros::Rate loop_rate(10);
	int counter = 0;
	while (counter <= QUIT_CMD_MAX_COUNTER){
		m_cmd_pub.publish(cmd);
		loop_rate.sleep();
		counter++;
	}
	ros::shutdown();
}




