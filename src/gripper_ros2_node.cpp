#include <gripper_ros2/gripper_node.h>


gripperNode::gripperNode(rclcpp::Node::SharedPtr node_ptr,rclcpp::Node::SharedPtr server_node_ptr) : node_(node_ptr),
																																																		server_node_(server_node_ptr),
															 									             																				gripper_object_()
{
	gripper_on_server_=server_node_->create_service<std_srvs::srv::SetBool>("on", std::bind(&gripperNode::gripperOnCallback,this,std::placeholders::_1,std::placeholders::_2));
	gripper_off_server_=server_node_->create_service<std_srvs::srv::SetBool>("off", std::bind(&gripperNode::gripperOffCallback,this,std::placeholders::_1,std::placeholders::_2));
	grip_item_server_=server_node_->create_service<std_srvs::srv::SetBool>("grip_status", std::bind(&gripperNode::gripStatusCallback,this,std::placeholders::_1,std::placeholders::_2));
}

void gripperNode::gripperOnCallback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
          std::shared_ptr<std_srvs::srv::SetBool::Response>      response)
{
	if(request->data)
	{
		RCLCPP_INFO(node_->get_logger(),"Turning on the gripper");
		response->success = gripper_object_.gripperOn();
	}
	else
	{
		response->success = false;
	}


}

void gripperNode::gripperOffCallback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
          std::shared_ptr<std_srvs::srv::SetBool::Response>      response)
{
	if(request->data)
	{
		RCLCPP_INFO(node_->get_logger(),"Turning off the gripper");
		response->success = gripper_object_.gripperOff();
	}

	else
	{
		response->success = false;
	}

}

void gripperNode::gripStatusCallback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
          std::shared_ptr<std_srvs::srv::SetBool::Response>      response)
{
	if(request->data)
	{
		RCLCPP_INFO(node_->get_logger(),"Getting gripped object status");
		response->success = gripper_object_.gripStatus();
	}
	else
	{
		response->success = false;
	}

}

void gripperNode::enableGripper()
{
	while(rclcpp::ok())
	{
		gripper_object_.writeToGripper(gripper_object_.gripperActivatedata);
		std::this_thread::sleep_for(std::chrono::milliseconds(10ms));
	}
}



int main(int argc, char **argv)
{
	std::cout<<"testing"<<std::endl;
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  std::shared_ptr<rclcpp::Node> node_ptr = rclcpp::Node::make_shared("gripper_node",node_options);
	std::shared_ptr<rclcpp::Node> server_node_ptr = rclcpp::Node::make_shared("gripper_server_node",node_options);

	rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node_ptr);
  executor.add_node(server_node_ptr);
  std::thread([&executor]() { executor.spin(); }).detach();

	gripperNode gripper_node_object(node_ptr,server_node_ptr);
	gripper_node_object.enableGripper();

	return 0;
}
