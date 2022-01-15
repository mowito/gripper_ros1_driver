#include <gripper_ros2/gripper_node.h>


gripperNode::gripperNode(rclcpp::Node::SharedPtr node_ptr) : node_(node_ptr),
															 									             gripper_object_()
{
	gripper_on_server_=node_->create_service<std_srvs::srv::SetBool>("on", std::bind(&gripperNode::gripperOnCallback,this,std::placeholders::_1,std::placeholders::_2));
	gripper_off_server_=node_->create_service<std_srvs::srv::SetBool>("off", std::bind(&gripperNode::gripperOffCallback,this,std::placeholders::_1,std::placeholders::_2));
  // while(1)
  // {
  //   gripper_object_.gripperOn();
  // }

}

void gripperNode::gripperOnCallback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
          std::shared_ptr<std_srvs::srv::SetBool::Response>      response)
{
	RCLCPP_INFO(node_->get_logger(),"Turning on the gripper");	
	response->success = gripper_object_.gripperOn();

}

void gripperNode::gripperOffCallback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
          std::shared_ptr<std_srvs::srv::SetBool::Response>      response)
{
	RCLCPP_INFO(node_->get_logger(),"Turning off the gripper");	
	response->success = gripper_object_.gripperOff();
}



int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  std::shared_ptr<rclcpp::Node> node_ptr = rclcpp::Node::make_shared("gripper_node",node_options);
    
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node_ptr);
  std::thread([&executor]() { executor.spin(); }).detach();

  gripperNode gripper_node_object(node_ptr);

}