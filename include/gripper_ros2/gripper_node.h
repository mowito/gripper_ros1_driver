#include <gripper_ros2/gripper_ros2.h>

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
    * @param request and response 
    */
	void gripperOnCallback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
          std::shared_ptr<std_srvs::srv::SetBool::Response>      response);
	 /**
    * @brief  /off service request callback
    * @param request and response 
    */
	void gripperOffCallback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
          std::shared_ptr<std_srvs::srv::SetBool::Response>      response);
	 /**
    * @brief  /get ros params from config files
    */
    void getRosParams();
private:
	rclcpp::Node::SharedPtr node_;
	gripperCommunication gripper_object_;

};