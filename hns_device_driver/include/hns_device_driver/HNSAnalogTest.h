/*
 * HNSAnalogTest.h
 *
 *  Created on: Jun 15, 2020
 *      Author: Wonseok Oh
 */

#ifndef HNS_GAS_DEVICE_HNS_DEVICE_DRIVER_INCLUDE_HNS_DEVICE_DRIVER_HNSANALOGTEST_H_
#define HNS_GAS_DEVICE_HNS_DEVICE_DRIVER_INCLUDE_HNS_DEVICE_DRIVER_HNSANALOGTEST_H_

#include <hns_msgs/HNSCommand.h>
#include <hns_msgs/HNSState.h>
#include <hns_msgs/ethanolsensor.h>
#include <std_msgs/String.h>
#include <ros/ros.h>
#include "serial/serial.h"
#include <string>
#include <cstdint>

namespace hns_analog_test {

class HNSAnalogTest {
public:
	HNSAnalogTest();
	~HNSAnalogTest();
	void cmdCallback(const std_msgs::String msg);
	void readData(const ros::TimerEvent& event);

private:
	hns_msgs::HNSState m_state;
	serial::Serial m_serial;
	ros::Timer m_timer_read;
	ros::Publisher m_state_pub;
	ros::Publisher m_data_pub;
	ros::Subscriber m_input_sub;
};

}



#endif /* HNS_GAS_DEVICE_HNS_DEVICE_DRIVER_INCLUDE_HNS_DEVICE_DRIVER_HNSANALOGTEST_H_ */
