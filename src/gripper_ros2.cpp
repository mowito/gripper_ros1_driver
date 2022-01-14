#include <gripper_ros2/gripper_ros2.h>

gripperCommunication::gripperCommunication()
{
	std::cout<<"inside gripperCommunication"<<std::endl;
	RCLCPP_INFO(rclcpp::get_logger(),"adasd");

	modbus_object_ = modbus_new_rtu("/dev/ttyTool", 115200, 'N', 8, 1);
	modbus_set_slave(modbus_object_,9);
  	
  	while(modbus_connect(modbus_object_) == -1) {
    std::cout<<"modbus connection to gripper failed. trying again"<<std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(500ms));   
  	}

  	std::cout<<"modbus connection established"<<std::endl;
  	//activate gripper
  	writeToGripper(gripperActivatedata); 

}

void gripperCommunication::writeToGripper(gripperInputData &data)
{
	uint16_t src[4];
	int *data_temp;
	convertGripperInputDataToArray(data,data_temp);
	for(int i=0;i<4;i++)
		src[i] = (data_temp[2*i] << 8)+ data_temp[2*i+1]; 
	modbus_write_registers(modbus_object_,0x03E8,4,src); 
	
}

void gripperCommunication::readFromGripper(gripperInputData &data)
{

}
void gripperCommunication::convertGripperInputDataToArray(gripperInputData &data,int *data_temp)
{
	data_temp = (int*)malloc(7 * sizeof(int));
	data_temp[0] = data.rACT;
	data_temp[1] = data.rMOD;
	data_temp[2] = data.rGTO;
	data_temp[3] = data.rATR;
	data_temp[4] = data.rPR;
	data_temp[5] = data.rSP;
	data_temp[6] = data.rFR;
	data_temp[8] = 0; //for easy iteration of for loop
}

bool gripperCommunication::gripperOn()
{
	writeToGripper(gripperOndata);
}

bool gripperCommunication::gripperOff()
{
	writeToGripper(gripperOffdata);
}