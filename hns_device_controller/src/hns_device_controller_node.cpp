/*
 * hns_device_controller_node.cpp
 *
 *  Created on: Apr 17, 2020
 *      Author: morin
 */


#include <hns_device_controller/HNSDeviceController.h>

using namespace hns_device_controller;
using namespace ros;

int main(int argc, char **argv){
	init(argc, argv, "hns_device_controller_node", init_options::NoSigintHandler);
	HNSDeviceController hns_device_controller;
	ros::spin();
	return 0;
}
