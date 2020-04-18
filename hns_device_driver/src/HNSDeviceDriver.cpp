/*
 * HNSDeviceDriver.cpp
 *
 *  Created on: Apr 17, 2020
 *      Author: Wonseok Oh
 */

#include <hns_device_driver/HNSDeviceDriver.h>

using namespace hns_device_driver;
using namespace hns_msgs;
using namespace std;

HNSDeviceDriver::HNSDeviceDriver(){
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
	m_input_sub = nh.subscribe<hns_msgs::HNSCommand>("cmd_HNS", 1, &HNSDeviceDriver::commandCallback, this);
	m_state_pub = nh.advertise<hns_msgs::HNSState>("state", 1);
	m_isInit = false;
	m_timer = nh.createTimer(ros::Duration(0.1), &HNSDeviceDriver::cycleManager, this);
	for (int i = HNSCommand::RESET; i <= HNSCommand::VALVE_4; i++){
		m_state.device[i] = false;
	}
}

HNSDeviceDriver::~HNSDeviceDriver(){}

void HNSDeviceDriver::commandCallback(hns_msgs::HNSCommand msg){
	// Command validity check
	if (msg.valve_num < HNSCommand::RESET || msg.valve_num > HNSCommand::VALVE_4 ||
			msg.option_num < 0 || msg.option_num > 10){
		ROS_ERROR("HNSDeviceDriver: commandCallback: received topic msg is not in proper form");
		return;
	}

	m_lastInputTime = ros::Time::now();
	// if the input command is different with previous one or not initialized,
	if (!m_isInit || msg.option_num != m_state.cmd.option_num || msg.valve_num != m_state.cmd.valve_num){
		executeFirstCommand(msg, m_isInit);
		return;
	}

	// If incoming command is same with previous one, do nothing
	return;
}

void HNSDeviceDriver::executeFirstCommand(HNSCommand cmd, bool isInit){
	m_firstInputTime = ros::Time::now();
	if (!isInit){
		// If cmd is opening cmd, send valve open serial command. Report the time & return
		if (cmd.option_num != 10 && cmd.valve_num != HNSCommand::RESET){
			m_serial.write(string("0") + to_string(2*m_state.cmd.valve_num-1) + string("*"));
			m_lastCtrlTime = ros::Time::now();
			ROS_INFO("First execution time: %f", (m_lastCtrlTime-m_firstInputTime).toSec());
			m_state.device[0] = true;
			m_state.device[cmd.valve_num] = true;
			m_state.cmd = cmd;
			m_isInit = true;
			return;
		}
		// else, do nothing (return)
		return;
	}

	// If initialized,
	else {
		// Set a command to close old valve
		string close_old_valve_cmd = to_string(2*m_state.cmd.valve_num);
		string new_valve_cmd;
		m_state.device[m_state.cmd.valve_num] = false;
		ROS_INFO("m_state.cmd.valve_num: %d", m_state.cmd.valve_num);

		// If cmd is opening command, set a new command to open the valve
		if (cmd.option_num != 10  && cmd.valve_num != HNSCommand::RESET){

			new_valve_cmd = to_string(2*cmd.valve_num-1);
			m_state.device[cmd.valve_num] = true;

			// if valve is closed before, add "pump on" to the new command
			if (m_state.device[0] == false) {
				new_valve_cmd = new_valve_cmd + string("*");
				m_state.device[0] = true;
			}
		}

		// else if cmd is closing and pump is open, set the new command to pump off
		else if (m_state.device[0] == true){
			new_valve_cmd = string("*");
			m_state.device[0] = false;
		}

		m_serial.write(close_old_valve_cmd + new_valve_cmd);
		m_lastCtrlTime = ros::Time::now();
		ROS_INFO("new cycle time: %f", (m_lastCtrlTime-m_firstInputTime).toSec());
		m_state.cmd = cmd;
		return;
	}
}

void HNSDeviceDriver::cycleManager(const ros::TimerEvent& event){
	// if not initialized, do nothing
	if (!m_isInit) return;

	m_state_pub.publish(m_state);
	ros::Time now = ros::Time::now();
	if (ros::Time::now() - m_lastInputTime > ros::Duration(3)){
		m_serial.write(string("0"));
		m_lastCtrlTime = ros::Time::now();
		ROS_INFO("Reset time: %f", (m_lastCtrlTime-m_firstInputTime).toSec());
		for (int i = 0; i <= HNSCommand::VALVE_4; i++){
			m_state.device[i] = false;
		}
		return;
	}

	else {
		if (m_state.device[0] == true){
			if (now - m_lastCtrlTime >= ros::Duration(10 - m_state.cmd.option_num) &&
					m_state.cmd.option_num != 0){
				m_serial.write(to_string(2*m_state.cmd.valve_num) + string("*"));
				m_lastCtrlTime = ros::Time::now();
				ROS_INFO("Off time: %f", (m_lastCtrlTime-m_firstInputTime).toSec());
				m_state.device[m_state.cmd.valve_num] = false;
				m_state.device[0] = false;
				return;
			}
			else return;
		}

		else {
			if (now - m_lastCtrlTime >= ros::Duration(m_state.cmd.option_num) &&
					m_state.cmd.option_num != 10){
				m_serial.write(to_string(2*m_state.cmd.valve_num-1) + string("*"));
				m_lastCtrlTime = ros::Time::now();
				ROS_INFO("On time: %f", (m_lastCtrlTime-m_firstInputTime).toSec());
				m_state.device[m_state.cmd.valve_num] = true;
				m_state.device[0] = true;
				return;
			}
			else return;
		}
	}
}
