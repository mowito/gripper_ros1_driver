#include <modbus.h>

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
struct gripperOutputData{
	uint8_t rACT;
	uint8_t rMOD;
	uint8_t rGTO;
	uint8_t rSTA;
	uint8_t rOBJ;
	uint8_t rFLT;
	uint8_t rPR;
	uint8_t rPO;
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
	void readFromGripper(gripperOutputData &data);
	/**
	 * @brief turn on gripper
	 * @param struct containing all register input values
	 */
	bool gripperOn(uint8_t timeout);
	/**
	 * @brief turn off gripper
	 */
	bool gripperOff();
	/**
	* @brief get gripped object status
	*/
	bool gripStatus();
	
	//data struct for turning on
	gripperInputData gripperOndata = {
		1,//rACT
		1,//rMOD
		1,//rGTO
		0,//rATR
		0,//rPR Grip to 78% of vacuum (Max possible?)
		100,//rSP timeout 10s
		255//- rFr Object will be detected when vacuum level reaches 20%.
	};
	//data struct for activate
	gripperInputData gripperActivatedata = {
		1,//rACT
		0,//rMOD
		0,//rGTO
		0,//rATR
		0,//rPR Grip to 78% of vacuum (Max possible?)
		0,//rSP timeout 1s
		0//- rFr Object will be detected when vacuum level reaches 20%.
	};
	//data struct for release
	gripperInputData gripperOffdata = {
		1,//rACT
		1,//rMOD // try automatic for releasing
		1,//rGTO
		0,//rATR
		255,//rPR immediate release
		100,//rSP timeout 10s
		255//- rFr Object will be detected when vacuum level reaches 20%.
	};


private:
	modbus_t *modbus_object_;





};
