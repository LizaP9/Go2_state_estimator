#include <foot_contacts.h>

int main(int argc, char** argv )
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FootContacts>());
  rclcpp::shutdown();
  return 0;
}
