#include <gripper_ros2/gripper_ros2_node.h>


gripperNode::gripperNode(rclcpp::Node::SharedPtr node_ptr) : node_(node_ptr),
															 									             gripper_object_()
{
	//server clients
	gripper_on_server_=node_->create_service<std_srvs::srv::SetBool>("on", std::bind(&gripperNode::gripperOnCallback,this,std::placeholders::_1,std::placeholders::_2));
	gripper_off_server_=node_->create_service<std_srvs::srv::SetBool>("off", std::bind(&gripperNode::gripperOffCallback,this,std::placeholders::_1,std::placeholders::_2));
	grip_item_server_=node_->create_service<std_srvs::srv::SetBool>("grip_status", std::bind(&gripperNode::gripStatusCallback,this,std::placeholders::_1,std::placeholders::_2));
	//rosparam for gripper on timeout
	rclcpp::Parameter on_timeout_parameter;
	node_->get_parameter_or("on_timeout",on_timeout_parameter,rclcpp::Parameter("on_timeout",100));
	timeout_ = uint8_t(on_timeout_parameter.as_int());
	//Testing gripper by turning it on for 1sec
	uint8_t timeout = 10;
	RCLCPP_INFO(node_->get_logger(),"Gripper testing, %s",(gripper_object_.gripperOn(timeout)) ? "Working" : "Not working");
}


void gripperNode::gripperOnCallback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
          std::shared_ptr<std_srvs::srv::SetBool::Response>      response)
{

	if(request->data)
	{
		RCLCPP_INFO(node_->get_logger(),"Turning on the gripper");
		response->success = gripper_object_.gripperOn(timeout_);
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


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  std::shared_ptr<rclcpp::Node> node_ptr = rclcpp::Node::make_shared("gripper_node",node_options);

	gripperNode gripper_node_object(node_ptr);
	rclcpp::spin(node_ptr);
	rclcpp::shutdown();

	return 0;
}
