/*
 * HNSAnalogTest.cpp
 *
 *  Created on: Jun 15, 2020
 *      Author: Wonseok Oh
 */

#include <hns_device_driver/HNSAnalogTest.h>

using namespace hns_analog_test;
using namespace hns_msgs;
using namespace std_msgs;
using namespace std;

HNSAnalogTest::HNSAnalogTest(){
	ros::NodeHandle private_nh("~");
	ros::NodeHandle nh;
	string port;
	int baudrate;
	int timeout;
	private_nh.param("port", port, string("/dev/ttyACM0"));
	private_nh.param("baudrate", baudrate, 9600);
	private_nh.param("timeout", timeout, 1000);
	baudrate = static_cast<uint32_t>(baudrate);
	timeout = static_cast<uint32_t>(timeout);
 	serial::Timeout timeout_class = serial::Timeout::simpleTimeout(timeout);
 	m_serial.setPort(port);
 	m_serial.setBaudrate(baudrate);
 	m_serial.setTimeout(timeout_class);
	m_serial.open();
	cout << "Is the serial port open? ";
	if (m_serial.isOpen()) cout << "Yes." << endl;
	else cout << "No." << endl;
	m_input_sub = nh.subscribe<std_msgs::Int64>("cmd_HNS", 1, &HNSAnalogTest::cmdCallback, this);
	m_state_pub = nh.advertise<hns_msgs::HNSState>("state", 1);
	m_data_pub = nh.advertise<hns_msgs::ethanolsensor>("ethanol_data", 100);
	m_timer_read = nh.createTimer(ros::Duration(0.1), &HNSAnalogTest::readData, this);

	for (int i = HNSCommand::RESET; i <= HNSCommand::VALVE_4; i++){
		m_state.device[i] = false;
	}
}

HNSAnalogTest::~HNSAnalogTest(){}

void HNSAnalogTest::cmdCallback(const std_msgs::Int64 msg){
	m_serial.write(to_string(msg.data));
	return;
}

void HNSAnalogTest::readData(const ros::TimerEvent& event){
	string buffer = m_serial.readline();
	string::size_type n = buffer.find('@');
	if (n == string::npos){
		return;
	}
	else {
		stringstream data(buffer.substr(n+1));
		int num_data;
		data >> num_data;
		ethanolsensor sensor_data;
		sensor_data.CH3CH2OH = num_data;
		m_data_pub.publish(sensor_data);
	}

}
