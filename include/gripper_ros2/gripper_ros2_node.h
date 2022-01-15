#include <gripper_ros2/robotiq_gripper_driver.h>

class gripperNode
{
public:
	/**
    * @brief  Constructor for the class
    * @param node ptr object
    */
    gripperNode(rclcpp::Node::SharedPtr node_ptr);
    /**
    * @brief  /on service request callback
    * @param1 request
    *  @param2 response
    */
	void gripperOnCallback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
          std::shared_ptr<std_srvs::srv::SetBool::Response>      response);
	 /**
    * @brief  /off service request callback
    * @param1 request
    *  @param2 response
    */
	void gripperOffCallback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
          std::shared_ptr<std_srvs::srv::SetBool::Response>      response);
    /**
    * @brief /grip_status service gripperOnCallback
    * @param1 request
    *@param2 Response
    **/
  void gripStatusCallback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
              std::shared_ptr<std_srvs::srv::SetBool::Response>      response);



private:
	rclcpp::Node::SharedPtr node_;
	rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr gripper_on_server_;
	rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr gripper_off_server_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr grip_item_server_;
  gripperCommunication gripper_object_;
  gripperInputData current_input_data_;
  uint8_t timeout_ ;

};
