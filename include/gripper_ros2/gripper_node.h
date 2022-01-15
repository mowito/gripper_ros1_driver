#include <gripper_ros2/gripper_ros2.h>

class gripperNode
{
public:
	/**
    * @brief  Constructor for the class
    * @param node ptr object
    */
    gripperNode(rclcpp::Node::SharedPtr node_ptr,rclcpp::Node::SharedPtr server_node_ptr);
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
	 /**
    * @brief  /get ros params from config files
    */
  void getRosParams();
  /**
   * @brief enable gripper by kepping rACT 1 always
   */
   void enableGripper();


private:
	rclcpp::Node::SharedPtr node_;
  rclcpp::Node::SharedPtr server_node_;
	rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr gripper_on_server_;
	rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr gripper_off_server_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr grip_item_server_;
  gripperCommunication gripper_object_;

};
