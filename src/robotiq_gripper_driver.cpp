#include <gripper_ros2/robotiq_gripper_driver.h>

gripperCommunication::gripperCommunication()
{
	modbus_object_ = modbus_new_rtu("/dev/ttyTool", 115200, 'N', 8, 1);
	modbus_set_slave(modbus_object_,9);

  	while(modbus_connect(modbus_object_) == -1) {
    std::cout<<"modbus connection to gripper failed. trying again"<<std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(500ms));
  	}

  	std::cout<<"modbus connection established"<<std::endl;

}

void gripperCommunication::writeToGripper(gripperInputData &data)
{
	uint16_t src[3];
	uint16_t mid_data_structure[6];
	
	mid_data_structure[0] = data.rACT + (data.rMOD << 1) + (data.rGTO << 3) + (data.rATR << 4);
	mid_data_structure[1] = 0;
	mid_data_structure[2] = 0;
	mid_data_structure[3] = data.rPR;
	mid_data_structure[4] = data.rSP;
	mid_data_structure[5] = data.rFR;
	for(int i=0;i<3;i++)
		src[i] = (mid_data_structure[2*i] << 8)+ mid_data_structure[2*i+1];

	modbus_write_registers(modbus_object_,0x03E8,3,src);
}

void gripperCommunication::readFromGripper(gripperOutputData &data)
{
	uint16_t response[3];
	uint16_t data_temp[12];
	modbus_read_registers(modbus_object_,0x07D0,3,response);
	for(int i=0;i<6;i++)
	{
		data_temp[2*i] = ((response[i] & 0xFF00) >> 8);
		data_temp[2*i+1] = (response[i]& 0x00FF);
	}

	data.rACT = (data_temp[0] >> 0) & 0x01;
  data.rMOD = (data_temp[0] >> 1) & 0x03;
  data.rGTO = (data_temp[0] >> 3) & 0x01;
  data.rSTA = (data_temp[0] >> 4) & 0x03;
  data.rOBJ = (data_temp[0] >> 6) & 0x03;
  data.rFLT = data_temp[2];
  data.rPR = data_temp[3];
  data.rPO = data_temp[4];

}

bool gripperCommunication::gripperOn(uint8_t timeout)
{
	writeToGripper(gripperOffdata);
	gripperOndata.rSP = timeout;
	writeToGripper(gripperOndata);
	return(true);
}

bool gripperCommunication::gripperOff()
{
	writeToGripper(gripperOffdata);
	return(true);
}
bool gripperCommunication::gripStatus()
{
	gripperOutputData data;
	readFromGripper(data);
	return(data.rOBJ == 1 || data.rOBJ == 2);
}
