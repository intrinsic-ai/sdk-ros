#include <rclcpp/rclcpp.hpp>
#include <iostream>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("test_service_node");
  std::cout << "Test service node started!" << std::endl;
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
