/*
 * hns_device_driver_node.cpp
 *
 *  Created on: Apr 17, 2020
 *      Author: morin
 */


#include <hns_device_driver/HNSAnalogTest.h>

using namespace hns_analog_test;
using namespace ros;

int main(int argc, char **argv){
	init(argc, argv, "hns_analog_test_node");
	HNSAnalogTest hns_analog_test;
	ros::spin();
	return 0;
}
