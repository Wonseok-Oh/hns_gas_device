/*
 * HNSDeviceDriver.h
 *
 *  Created on: Apr 17, 2020
 *      Author: morin
 */

#ifndef HNS_GAS_DEVICE_HNS_DEVICE_DRIVER_INCLUDE_HNS_DEVICE_DRIVER_HNSDEVICEDRIVER_H_
#define HNS_GAS_DEVICE_HNS_DEVICE_DRIVER_INCLUDE_HNS_DEVICE_DRIVER_HNSDEVICEDRIVER_H_

#include <hns_msgs/HNSCommand.h>
#include <hns_msgs/HNSState.h>
#include <hns_msgs/ethanolsensor.h>
#include <ros/ros.h>
#include "serial/serial.h"
#include <string>
#include <cstdint>


namespace hns_device_driver {

class HNSDeviceDriver {
public:
	HNSDeviceDriver();
	~HNSDeviceDriver();
	void commandCallback(hns_msgs::HNSCommand msg);
	void cycleManager(const ros::TimerEvent& event);
	void executeFirstCommand(hns_msgs::HNSCommand cmd, bool isInit);
	void readData(const ros::TimerEvent& event);

private:
	hns_msgs::HNSState m_state;
	serial::Serial m_serial;
	bool m_isInit;
	ros::Timer m_timer;
	ros::Timer m_timer_read;
	ros::Time m_lastCtrlTime;
	ros::Time m_lastInputTime;
	ros::Time m_firstInputTime;
	ros::Publisher m_state_pub;
	ros::Publisher m_data_pub;
	ros::Subscriber m_input_sub;

};

}



#endif /* HNS_GAS_DEVICE_HNS_DEVICE_DRIVER_INCLUDE_HNS_DEVICE_DRIVER_HNSDEVICEDRIVER_H_ */
