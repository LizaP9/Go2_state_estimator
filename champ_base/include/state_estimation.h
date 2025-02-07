#ifndef STATE_ESTIMATION_H
#define STATE_ESTIMATION_H

#include "rclcpp/rclcpp.hpp"
#include <champ_msgs/msg/contacts_stamped.hpp>
#include <champ/odometry/odometry.h>
#include <champ/kinematics/kinematics.h>
#include <champ/utils/urdf_loader.h>
#include <tf2/LinearMath/Quaternion.h>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <sensor_msgs/msg/imu.hpp>
#include "unitree_go/msg/low_state.hpp"
#include <Eigen/Dense>

class StateEstimation : public rclcpp::Node
{
public:
    StateEstimation();

private:
    // Subscribers
    rclcpp::Subscription<champ_msgs::msg::ContactsStamped>::SharedPtr foot_contacts_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
    rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr lowstate_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr dlio_odom_subscriber_;

    // Publishers
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr footprint_to_odom_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr base_to_footprint_publisher_;

    // Timers
    rclcpp::TimerBase::SharedPtr odom_data_timer_;
    rclcpp::TimerBase::SharedPtr base_pose_timer_;

    // Transform broadcaster
    std::unique_ptr<tf2_ros::TransformBroadcaster> base_broadcaster_;

    // State variables
    champ::Velocities current_velocities_;
    geometry::Transformation current_foot_positions_[4];
    geometry::Transformation target_foot_positions_[4];
    champ::Kinematics kinematics_;

    float x_pos_;
    float y_pos_;
    float heading_ = 0.0;

    // IMU orientation variables
    float yaw_ = 0.0;                  // Current yaw angle
    float initial_yaw_ = 0.0;          // Initial yaw value
    bool is_initial_yaw_set_ = false;  // Flag to check if initial yaw is set

    // DLIO odometry data variables
    double dlio_x_ = 0.0;
    double dlio_y_ = 0.0;
    double dlio_heading_ = 0.0;
    bool dlio_data_received_ = false;  // Flag to ensure data has been received

    // Kalman filter variables for position
    Eigen::Vector3d state_;      // State vector [x, y, heading]
    Eigen::Matrix3d P_;          // Error covariance matrix
    Eigen::Matrix3d Q_;          // Process covariance
    Eigen::Matrix3d R_;          // Measurement covariance

    // Kalman filter variables for orientation
    Eigen::Vector2d state_orientation_;    // State vector for [roll, pitch]
    Eigen::Matrix2d P_orientation_;        // Error covariance matrix
    Eigen::Matrix2d Q_orientation_;        // Process covariance
    Eigen::Matrix2d R_orientation_;        // Measurement covariance

    // IMU Roll and Pitch
    double imu_roll_ = 0.0;
    double imu_pitch_ = 0.0;

    // Time variables
    rclcpp::Time last_vel_time_;
    rclcpp::Time last_sync_time_;
    rclcpp::Clock clock_;

    // Other variables
    sensor_msgs::msg::Imu::SharedPtr last_imu_;
    champ::GaitConfig gait_config_;
    champ::QuadrupedBase base_;
    champ::Odometry odometry_;

    std::vector<std::string> joint_names_;
    std::string base_name_;
    std::string node_namespace_;
    std::string odom_frame_;
    std::string base_footprint_frame_;
    std::string base_link_frame_;
    bool orientation_from_imu_;

    // Foot contact storage
    std::array<bool, 4> foot_contacts_;

    float current_joint_positions[12];

    // Callback functions
    void publishFootprintToOdom_();
    void lowstate_callback_(const unitree_go::msg::LowState::SharedPtr msg);
    void foot_contacts_callback_(const champ_msgs::msg::ContactsStamped::SharedPtr msg);
    void imu_callback_(const sensor_msgs::msg::Imu::SharedPtr msg);
    void dlio_odom_callback_(const nav_msgs::msg::Odometry::SharedPtr msg);

    // Orientation computation
    void computeOrientationFromKinematics_();

    // Kalman filter update function
    void kalmanFilterUpdate(const Eigen::Vector3d& measurement, double dt);


    // Function to get yaw angle from Quaternion
    double getYawFromQuaternion(const tf2::Quaternion& quat);

    double normalizeAngle(double angle);

    float yaw_rate_ = 0.0;

    float filtered_yaw_rate_; // Для фильтрованной угловой скорости
    float previous_yaw_rate_; // Для хранения предыдущей угловой скорости

    tf2::Quaternion initial_orientation_; // Начальная ориентация из IMU
    bool is_initial_orientation_set_ = false; // Флаг установки начальной ориентации

    tf2::Quaternion current_orientation_quat_;

    tf2::Quaternion initial_orientation_quat_; // Для хранения начального ориентационного кватерниона

    float roll_;   // Roll из кинематики
    float pitch_;  // Pitch из кинематики

    float robot_height_ = 0.0; // Высота робота

};

#endif // STATE_ESTIMATION_H
