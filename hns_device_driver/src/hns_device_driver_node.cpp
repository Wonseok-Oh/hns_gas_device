/*
 * hns_device_driver_node.cpp
 *
 *  Created on: Apr 17, 2020
 *      Author: morin
 */


#include <hns_device_driver/HNSDeviceDriver.h>

using namespace hns_device_driver;
using namespace ros;

int main(int argc, char **argv){
	init(argc, argv, "hns_device_driver_node");
	HNSDeviceDriver hns_device_driver;
	ros::spin();
	return 0;
}
