#ifndef FOOT_CONTACTS_H
#define FOOT_CONTACTS_H


#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <champ_msgs/msg/contacts_stamped.hpp>
#include "unitree_go/msg/low_state.hpp"

class FootContacts : public rclcpp::Node
{

    rclcpp::Clock clock_;
public:
    FootContacts();

private:
    void foot_force_callback(const unitree_go::msg::LowState::SharedPtr msg); 
    void publish_data();

    rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr foot_force_subscriber_;

    rclcpp::Publisher<champ_msgs::msg::ContactsStamped>::SharedPtr foot_contacts_publisher_;

    rclcpp::TimerBase::SharedPtr timer_;

    std::array<int16_t, 4> foot_forces_;

};

#endif







