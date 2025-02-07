#include <state_estimation.h>
#include <Eigen/Dense>

// Определим пороги для обнаружения покоя
constexpr float LINEAR_VELOCITY_THRESHOLD = 0.01;  // Порог для линейной скорости
constexpr float ANGULAR_VELOCITY_THRESHOLD = 0.02; // Порог для угловой скорости

// Коэффициент для высокочастотного фильтра угловой скорости
constexpr float HIGH_PASS_ALPHA = 0.1;

// Convert ROS time to Champ time
champ::Odometry::Time rosTimeToChampTime(const rclcpp::Time& time)
{
    return time.nanoseconds() / 1000ul;
}

StateEstimation::StateEstimation()
    : Node("state_estimation_node", rclcpp::NodeOptions()
                                     .allow_undeclared_parameters(true)
                                     .automatically_declare_parameters_from_overrides(true)),
      clock_(*this->get_clock()),
      odometry_(base_, rosTimeToChampTime(clock_.now())),
      foot_contacts_{false, false, false, false}, // Initialize foot contacts as false
      kinematics_(base_),
      x_pos_(0.0), // Explicit initialization of coordinates
      y_pos_(0.0),
      heading_(0.0),
      yaw_(0.0),
      is_initial_yaw_set_(false),
      filtered_yaw_rate_(0.0),
      initial_yaw_(0.0),
      roll_(0.0),
      pitch_(0.0)
      //robot_height_(0.0) // Инициализация высоты
{
    //current_orientation_quat_.setValue(0.0, 0.0, 0.0, 1.0); // Единичный кватернион

    last_vel_time_ = clock_.now();
    last_sync_time_ = clock_.now();
    base_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // Subscribe to lowstate instead of joint_states
    lowstate_subscriber_ = this->create_subscription<unitree_go::msg::LowState>(
        "lowstate", 10, std::bind(&StateEstimation::lowstate_callback_, this, std::placeholders::_1));

    // Subscribe to foot contacts
    foot_contacts_subscriber_ = this->create_subscription<champ_msgs::msg::ContactsStamped>(
        "foot_contacts", 10, std::bind(&StateEstimation::foot_contacts_callback_, this, std::placeholders::_1));

    // Subscribe to DLIO odometry data
    dlio_odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/dlio/odom_node/odom", 10, std::bind(&StateEstimation::dlio_odom_callback_, this, std::placeholders::_1));

    // Check if robot is standing on all feet
    // bool all_feet_in_contact = std::all_of(
    //     foot_contacts_.begin(), foot_contacts_.end(),
    //     [](bool contact) { return contact; });

    // if (!all_feet_in_contact)
    // {
    // // Если не все ноги в контакте, задаём начальные значения
    //     robot_height_ = -0.0809;

    // // Создаём кватернион с компонентами: x = 0.0, y = 0.03, z = 0.0, w = sqrt(1 - 0.03^2)
    //     tf2::Quaternion init_quat(0.0, 0.03, 0.0, sqrt(1 + 0.03 * 0.03));

    // // Локальные переменные для извлечения углов
    //     double temp_roll, temp_pitch, temp_yaw;
    //     tf2::Matrix3x3(init_quat).getRPY(temp_roll, temp_pitch, temp_yaw); // Получаем roll, pitch, yaw

    //     roll_ = 0.0;        // Задаём начальный roll вручную
    //     pitch_ = temp_pitch; // Используем рассчитанный pitch
    //     yaw_ = temp_yaw;     // Устанавливаем начальный yaw

    //     RCLCPP_WARN(this->get_logger(),
    //             "Robot is not standing on all feet. Setting default height: %.4f, pitch: %.4f (rad), and roll: %.4f",
    //             robot_height_, pitch_, roll_);
    // }
    // else
    // {
    //     RCLCPP_INFO(this->get_logger(), "Robot is standing on all feet at initialization.");
    // }



    // Initialize Kalman filter
    state_ << 0.0, 0.0, 0.0;
    P_ = Eigen::Matrix3d::Identity() * 1e-3; // Initial error covariance
    Q_ = Eigen::Matrix3d::Identity() * 1e-4; // Process covariance
    R_ = Eigen::Matrix3d::Identity() * 1e-2; // Measurement covariance

    // Create odometry publisher
    base_to_footprint_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("estimater", 1);

    std::string urdf = "";

    orientation_from_imu_ = false;

    // Retrieve parameters
    this->get_parameter("links_map.base", base_name_);
    this->get_parameter("gait.odom_scaler", gait_config_.odom_scaler);
    this->get_parameter("orientation_from_imu", orientation_from_imu_);

    this->get_parameter("gait.pantograph_leg", gait_config_.pantograph_leg);
    this->get_parameter("gait.max_linear_velocity_x", gait_config_.max_linear_velocity_x);
    this->get_parameter("gait.max_linear_velocity_y", gait_config_.max_linear_velocity_y);
    this->get_parameter("gait.max_angular_velocity_z", gait_config_.max_angular_velocity_z);
    this->get_parameter("gait.com_x_translation", gait_config_.com_x_translation);
    this->get_parameter("gait.swing_height", gait_config_.swing_height);
    this->get_parameter("gait.stance_depth", gait_config_.stance_depth);
    this->get_parameter("gait.stance_duration", gait_config_.stance_duration);
    this->get_parameter("gait.nominal_height", gait_config_.nominal_height);
    this->get_parameter("urdf", urdf);

    // Subscribe to IMU data if orientation is from IMU
    if (orientation_from_imu_)
        imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/utlidar/imu", 1, std::bind(&StateEstimation::imu_callback_, this, std::placeholders::_1));

    base_.setGaitConfig(gait_config_);
    champ::URDF::loadFromString(base_, this->get_node_parameters_interface(), urdf);
    joint_names_ = champ::URDF::getJointNames(this->get_node_parameters_interface());

    node_namespace_ = this->get_namespace();
    if (node_namespace_.length() > 1)
    {
        node_namespace_.replace(0, 1, "");
        node_namespace_.push_back('/');
    }
    else
    {
        node_namespace_ = "";
    }

    odom_frame_ = node_namespace_ + "odom";
    base_footprint_frame_ = node_namespace_ + "base_footprint";
    base_link_frame_ = node_namespace_ + base_name_;

    std::chrono::milliseconds period(static_cast<int>(1000 / 50));

    odom_data_timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(period),
        std::bind(&StateEstimation::publishFootprintToOdom_, this));
}


void StateEstimation::lowstate_callback_(const unitree_go::msg::LowState::SharedPtr msg)
{
    last_sync_time_ = clock_.now();
    float current_joint_positions[12];

    for (size_t i = 0; i < 12; ++i)
    {
        current_joint_positions[i] = msg->motor_state[i].q;
    }

    base_.updateJointPositions(current_joint_positions);

    // Use inverse kinematics to calculate foot positions
    kinematics_.inverse(current_joint_positions, current_foot_positions_);

    for (size_t i = 0; i < foot_contacts_.size(); i++)
    {
        base_.legs[i]->in_contact(foot_contacts_[i]);
    }
}

void StateEstimation::foot_contacts_callback_(const champ_msgs::msg::ContactsStamped::SharedPtr msg)
{
    // Store contact states in foot_contacts_ in the correct order
    for (size_t i = 0; i < foot_contacts_.size(); ++i)
    {
        foot_contacts_[i] = msg->contacts[i];
    }
}



void StateEstimation::imu_callback_(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    tf2::Quaternion imu_quat(
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z,
        msg->orientation.w
    );

    double yaw_imu = getYawFromQuaternion(imu_quat);

    // Сохраняем начальный yaw при первом вызове
    if (!is_initial_yaw_set_)
    {
        initial_yaw_ = yaw_imu;
        is_initial_yaw_set_ = true;
        RCLCPP_INFO(this->get_logger(), "Initial IMU Yaw set to: %.4f", initial_yaw_);
    }

    // Обнуляем yaw относительно начального значения
    yaw_ = normalizeAngle(yaw_imu - initial_yaw_);

    // Только yaw используется, roll и pitch игнорируются
    RCLCPP_INFO(this->get_logger(), "IMU Yaw updated: %.4f", yaw_);
}










void StateEstimation::publishFootprintToOdom_()
{
    // Получаем текущие скорости
    odometry_.getVelocities(current_velocities_, rosTimeToChampTime(clock_.now()));
    rclcpp::Time current_time = clock_.now();
    double vel_dt = (current_time - last_vel_time_).nanoseconds() / 1e9;
    last_vel_time_ = current_time;

    // Обновляем угол heading с использованием yaw
    heading_ = normalizeAngle(yaw_);

    // Переводим скорости из локальной в глобальную систему координат
    double delta_x = (current_velocities_.linear.x * cos(heading_) - current_velocities_.linear.y * sin(heading_)) * vel_dt;
    double delta_y = (current_velocities_.linear.x * sin(heading_) + current_velocities_.linear.y * cos(heading_)) * vel_dt;

    // Обновляем глобальные координаты робота
    x_pos_ += delta_x;
    y_pos_ += delta_y;

    Eigen::Vector3d measurement;
    if (dlio_data_received_)
    {
        // Если данные DLIO получены, используем их
        measurement << dlio_x_, dlio_y_, dlio_heading_;
    }
    else
    {
        // Используем текущую позицию
        measurement << x_pos_, y_pos_, heading_;
    }

    // Обновляем фильтр Калмана
    kalmanFilterUpdate(measurement, vel_dt);

    // Обновляем переменные положения
    x_pos_ = state_(0);
    y_pos_ = state_(1);
    heading_ = state_(2);

    // Вычисляем ориентацию и высоту на основе кинематики
    computeOrientationFromKinematics_();

    base_.getFootPositions(current_foot_positions_);

    for (size_t i = 0; i < 4; ++i)
    {
        champ::Kinematics::forward(current_foot_positions_[i], *base_.legs[i],
                                   current_joint_positions[i * 3],
                                   current_joint_positions[i * 3 + 1],
                                   current_joint_positions[i * 3 + 2]);
    }

    float robot_height = 0.0, all_height = 0.0;
    int foot_in_contact = 0;
    geometry::Transformation touching_feet[4];
    bool no_contact = false;

    for (size_t i = 0; i < 4; ++i)
    {
        if (base_.legs[i]->in_contact())
        {
            robot_height += current_foot_positions_[i].Z();
            touching_feet[foot_in_contact] = current_foot_positions_[i];
            foot_in_contact++;
        }
        all_height += current_foot_positions_[i].Z();
    }

    if (foot_in_contact == 0)
    {
        no_contact = true;
        robot_height = all_height;
        foot_in_contact = 4;
        for (size_t i = 0; i < 4; ++i)
        {
            touching_feet[i] = current_foot_positions_[i];
        }
    }


    // Создаём сообщение одометрии
    nav_msgs::msg::Odometry pose_msg;
    pose_msg.header.stamp = current_time;
    pose_msg.header.frame_id = base_footprint_frame_;
    pose_msg.child_frame_id = base_link_frame_;

    // Устанавливаем позиции
    pose_msg.pose.pose.position.x = x_pos_;
    pose_msg.pose.pose.position.y = y_pos_;
    pose_msg.pose.pose.position.z = -(robot_height / static_cast<float>(foot_in_contact)); 

    // Создаем кватернион на основе yaw (IMU) и roll/pitch (кинематика)
    tf2::Quaternion quat;
    quat.setRPY(roll_, pitch_, yaw_); // Roll и pitch с кинематики, yaw с IMU
    quat.normalize();

    // Устанавливаем кватернион ориентации
    pose_msg.pose.pose.orientation.x = quat.x();
    pose_msg.pose.pose.orientation.y = quat.y();
    pose_msg.pose.pose.orientation.z = quat.z();
    pose_msg.pose.pose.orientation.w = quat.w();

    // Устанавливаем ковариацию положения
    // pose_msg.pose.covariance[0] = 0.001;
    // pose_msg.pose.covariance[7] = 0.001;
    // pose_msg.pose.covariance[14] = 0.001;
    // pose_msg.pose.covariance[21] = 0.0001;
    // pose_msg.pose.covariance[28] = 0.0001;
    // pose_msg.pose.covariance[35] = 0.017;

    // Устанавливаем скорости
    pose_msg.twist.twist.linear.x = current_velocities_.linear.x;
    pose_msg.twist.twist.linear.y = current_velocities_.linear.y;
    pose_msg.twist.twist.linear.z = 0.0;

    pose_msg.twist.twist.angular.x = 0.0;
    pose_msg.twist.twist.angular.y = 0.0;
    pose_msg.twist.twist.angular.z = current_velocities_.angular.z;

    // Устанавливаем ковариацию скоростей
    // pose_msg.twist.covariance[0] = 0.3;
    // pose_msg.twist.covariance[7] = 0.3;
    // pose_msg.twist.covariance[35] = 0.017;

    // Публикуем сообщение
    base_to_footprint_publisher_->publish(pose_msg);
}








void StateEstimation::dlio_odom_callback_(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    // Get position and orientation data from DLIO
    dlio_x_ = msg->pose.pose.position.x;
    dlio_y_ = msg->pose.pose.position.y;

    // Convert orientation from quaternion to yaw angle
    tf2::Quaternion quat(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w
    );

    dlio_heading_ = getYawFromQuaternion(quat);

    dlio_data_received_ = true; // Set flag indicating data has been received
}

double StateEstimation::getYawFromQuaternion(const tf2::Quaternion& quat)
{
    double roll, pitch, yaw;
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    return yaw;
}


void StateEstimation::kalmanFilterUpdate(const Eigen::Vector3d& measurement, double dt)
{
    // Шаг предсказания
    state_(0) += (current_velocities_.linear.x * cos(state_(2)) - current_velocities_.linear.y * sin(state_(2))) * dt;
    state_(1) += (current_velocities_.linear.x * sin(state_(2)) + current_velocities_.linear.y * cos(state_(2))) * dt;
    state_(2) += filtered_yaw_rate_ * dt; // Используем фильтрованную угловую скорость для предсказания yaw

    // Нормализуем heading_
    state_(2) = normalizeAngle(state_(2));

    // Предсказание ковариации ошибки
    P_ = P_ + Q_;

    // Проверка состояния покоя
    bool is_stationary = std::abs(current_velocities_.linear.x) < LINEAR_VELOCITY_THRESHOLD &&
                         std::abs(filtered_yaw_rate_) < ANGULAR_VELOCITY_THRESHOLD;

    // Уменьшаем ковариацию ориентации, если робот в покое
    if (is_stationary)
    {
        P_(2, 2) *= 0.0001; // Уменьшаем ковариацию yaw, чтобы подавить дрейф
    }

    // Шаг коррекции
    Eigen::Matrix3d H = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d S = H * P_ * H.transpose() + R_;
    Eigen::Matrix3d K = P_ * H.transpose() * S.inverse();

    // Обновление состояния на основе измерений
    Eigen::Vector3d y = measurement - H * state_;

    // Учитываем, что разница yaw должна быть нормализована
    y(2) = normalizeAngle(y(2));

    state_ = state_ + K * y;

    // Обновление ковариации ошибки
    P_ = (Eigen::Matrix3d::Identity() - K * H) * P_;

    // Нормализуем heading_ после коррекции
    state_(2) = normalizeAngle(state_(2));

    // Обновляем heading_ и yaw_
    heading_ = state_(2);
    //yaw_ = heading_;
}




double StateEstimation::normalizeAngle(double angle)
{
    // Приводим угол в диапазон от -π до π
    while (angle > M_PI)
        angle -= 2.0 * M_PI;
    while (angle < -M_PI)
        angle += 2.0 * M_PI;
    return angle;
}



void StateEstimation::computeOrientationFromKinematics_()
{
    // Проверяем наличие ног в контакте с поверхностью
    if (std::none_of(foot_contacts_.begin(), foot_contacts_.end(), [](bool contact) { return contact; }))
    {
        RCLCPP_WARN(this->get_logger(), "No feet in contact. Using default height, roll, and pitch.");
        roll_ = 0.0;  // Roll по умолчанию
        pitch_ = 0.0; // Pitch по умолчанию
        //robot_height_ = -0.0809; // Высота по умолчанию
        return;
    }

    // Получаем позиции ног через обратную кинематику
    base_.getFootPositions(current_foot_positions_);
    std::vector<Eigen::Vector3d> foot_positions;

    for (size_t i = 0; i < 4; ++i)
    {
        if (foot_contacts_[i]) // Учитываем только ноги в контакте с поверхностью
        {
            geometry::Transformation foot_pos = current_foot_positions_[i];
            foot_positions.emplace_back(foot_pos.X(), foot_pos.Y(), foot_pos.Z());
        }
    }

    if (foot_positions.empty())
    {
        RCLCPP_WARN(this->get_logger(), "No valid foot positions for kinematic computation.");
        return;
    }

    // Расчет высоты как максимального значения среди контактов
    // double max_height = 0.0;

    // for (const auto& pos : foot_positions)
    // {
    //     double height = std::max(0.075, -pos.z() + 0.03); // Минимальная высота 0.075, с учетом смещения 0.03
    //     max_height = std::max(max_height, height);
    // }

    //robot_height_ = -max_height;

    // Логируем рассчитанную высоту
    //RCLCPP_INFO(this->get_logger(), "Robot height calculated: %.4f", robot_height_);
}

