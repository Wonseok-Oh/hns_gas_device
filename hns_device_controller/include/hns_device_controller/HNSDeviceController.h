/*
 * HNSDeviceController.h
 *
 *  Created on: Apr 17, 2020
 *      Author: morin
 */

#ifndef HNS_GAS_DEVICE_HNS_DEVICE_CONTROLLER_INCLUDE_HNS_DEVICE_CONTROLLER_HNSDEVICECONTROLLER_H_
#define HNS_GAS_DEVICE_HNS_DEVICE_CONTROLLER_INCLUDE_HNS_DEVICE_CONTROLLER_HNSDEVICECONTROLLER_H_

#include <hns_msgs/HNSCommand.h>
#include <ros/ros.h>
#include <signal.h>


namespace hns_device_controller{
#define QUIT_CMD_MAX_COUNTER 30

class HNSDeviceController {
public:
	HNSDeviceController();
	~HNSDeviceController();
	void setCurrentCMD(const hns_msgs::HNSCommand msg);
	void timerCallback(const ros::TimerEvent& event);
	static void sigIntHandler(int sig);


private:
	hns_msgs::HNSCommand m_current_cmd;
	bool m_isInit;
	static ros::Publisher m_cmd_pub;
	ros::Subscriber m_input_sub;
	ros::Timer m_timer;
};

}



#endif /* HNS_GAS_DEVICE_HNS_DEVICE_CONTROLLER_INCLUDE_HNS_DEVICE_CONTROLLER_HNSDEVICECONTROLLER_H_ */
