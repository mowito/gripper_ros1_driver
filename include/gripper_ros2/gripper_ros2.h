#include <modbus.h>
#include "rclcpp/rclcpp.hpp"

#include <functional>
#include <chrono>
#include <memory>

#include <unistd.h>
#include <cstring>
#include <string.h>
#include <thread>
#include <future>

#include <signal.h>
#include <atomic>

#include <cstdio>
#include <iostream>
#include <stdexcept>
#include <string>
#include <array>
#include <std_srvs/srv/set_bool.hpp>
#include <geometry_msgs/msg/pose.h>
#include <functional>
using namespace std::chrono_literals;


struct gripperInputData{
	uint8_t rACT;
	uint8_t rMOD;
	uint8_t rGTO;
	uint8_t rATR;
	uint8_t rPR;
	uint8_t rSP;
	uint8_t rFR;
};

class gripperCommunication
{
public:
	/**
    * @brief  Constructor for the class
    */
	gripperCommunication();
	/**
	 * @brief writing something to modbus interface
	 * @param struct containing all register input values
	 */
	void writeToGripper(gripperInputData &data);
	/**
	 * @brief read something from modbus interface
	 */
	void readFromGripper(gripperInputData &data);
	/**
	 * @brief turn on gripper 
	 * @param struct containing all register input values
	 */
	bool gripperOn();
	/**
	 * @brief turn off gripper
	 */
	bool gripperOff();
	/**
	 * @brief utility funcn for converting struct to an array
	 */
	void convertGripperInputDataToArray(gripperInputData &data,int *data_temp);


private:
	std::shared_ptr<rclcpp::Node> node_;
	modbus_t *modbus_object_;
	gripperInputData gripperOndata = {
		1,//rACT
		1,//rMOD
		1,//rGTO
		0,//rATR
		0,//rPR Grip to 78% of vacuum (Max possible?)
		100,//rSP timeout 10s
		255//- rFr Object will be detected when vacuum level reaches 20%.
	};
	gripperInputData gripperActivatedata = {
		1,//rACT
		1,//rMOD
		1,//rGTO
		0,//rATR
		0,//rPR Grip to 78% of vacuum (Max possible?)
		10,//rSP timeout 1s
		255//- rFr Object will be detected when vacuum level reaches 20%.
	};
	gripperInputData gripperOffdata = {
		1,//rACT
		1,//rMOD // try automatic for releasing
		1,//rGTO
		0,//rATR
		255,//rPR immediate release
		100,//rSP timeout 10s
		255//- rFr Object will be detected when vacuum level reaches 20%.
	};
	
	


};



