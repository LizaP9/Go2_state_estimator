#include "foot_contacts.h"

FootContacts::FootContacts() : Node("foot_contacts")
{
    foot_force_subscriber_ = this->create_subscription<unitree_go::msg::LowState>(
            "lowstate",
            10,
            std::bind(&FootContacts::foot_force_callback, this, std::placeholders::_1)
        );

    foot_contacts_publisher_ = this->create_publisher<champ_msgs::msg::ContactsStamped>("/foot_contacts", 10);


    //foot_force_subscriber_.subscribe(reinterpret_cast<rclcpp::Node*>(this), "foot_force");
    
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100), std::bind(&FootContacts::publish_data, this));

}


void FootContacts::foot_force_callback(const unitree_go::msg::LowState::SharedPtr msg)
{
    for (size_t i = 0; i < 4; ++i) // 4 legs
    {
        foot_forces_[i] = msg->foot_force[i];
    }
}


void FootContacts::publish_data()
{

    champ_msgs::msg::ContactsStamped contacts_msg;
    contacts_msg.header.stamp = clock_.now();
    contacts_msg.contacts.resize(4);

    for (size_t i = 0; i < foot_forces_.size(); ++i)
    {
        contacts_msg.contacts[i] = foot_forces_[i] > 20;
    }

    foot_contacts_publisher_->publish(contacts_msg);
}
