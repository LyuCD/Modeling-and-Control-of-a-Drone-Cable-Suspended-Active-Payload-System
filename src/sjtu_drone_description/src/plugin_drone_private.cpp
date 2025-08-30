// Copyright 2024 Georg Novotny
//
// Licensed under the GNU GENERAL PUBLIC LICENSE, Version 3.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.gnu.org/licenses/gpl-3.0.en.html
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "gazebo_ros/conversions/builtin_interfaces.hpp"
#include <gazebo_ros/conversions/geometry_msgs.hpp>

#include "sjtu_drone_description/plugin_drone_private.h"
#include <Eigen/Dense>
namespace gazebo_plugins
{

DroneSimpleControllerPrivate::DroneSimpleControllerPrivate()
: m_timeAfterCmd(0.0)
  , navi_state(LANDED_MODEL)
  , m_posCtrl(false)
  , m_velMode(false)
  , odom_seq(0)
  , odom_hz(30)
  , last_odom_publish_time_(0.0)
{
}

DroneSimpleControllerPrivate::~DroneSimpleControllerPrivate() {}

void DroneSimpleControllerPrivate::Reset()
{
  // Reset the values of the controllers
  controllers_.roll.reset();
  controllers_.pitch.reset();
  controllers_.yaw.reset();
  controllers_.velocity_x.reset();
  controllers_.velocity_y.reset();
  controllers_.velocity_z.reset();

  // Set the force and torque acting on the drone to zero
  link->SetForce(ignition::math::Vector3<double>(0.0, 0.0, 0.0));
  link->SetTorque(ignition::math::v6::Vector3<double>(0.0, 0.0, 0.0));

  // Reset the state of the drone
  pose.Reset();
  velocity.Set();
  angular_velocity.Set();
  acceleration.Set();
  euler.Set();
}

// ROS Node configuration
void DroneSimpleControllerPrivate::InitSubscribers(
  std::string cmd_normal_topic_,
  std::string posctrl_topic_,
  std::string imu_topic_,
  std::string takeoff_topic_,
  std::string land_topic_,
  std::string reset_topic_,
  std::string switch_mode_topic_)
{
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();

  if (!cmd_normal_topic_.empty()) {
    cmd_subscriber_ = ros_node_->create_subscription<geometry_msgs::msg::Twist>(
      cmd_normal_topic_, qos,
      std::bind(&DroneSimpleControllerPrivate::CmdCallback, this, std::placeholders::_1));
  } else {
    RCLCPP_ERROR(ros_node_->get_logger(), "No cmd_topic defined!");
  }

  if (!posctrl_topic_.empty()) {
    posctrl_subscriber_ = ros_node_->create_subscription<std_msgs::msg::Bool>(
      posctrl_topic_, qos,
      std::bind(&DroneSimpleControllerPrivate::PosCtrlCallback, this, std::placeholders::_1));
  } else {
    RCLCPP_ERROR(ros_node_->get_logger(), "No position control defined!");
  }

  // subscribe imu
  if (!imu_topic_.empty()) {
    imu_subscriber_ = ros_node_->create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, qos,
      std::bind(&DroneSimpleControllerPrivate::ImuCallback, this, std::placeholders::_1));
  } else {
    RCLCPP_ERROR(ros_node_->get_logger(), "No imu topic defined!");
  }

  // subscribe command: take off command
  if (!takeoff_topic_.empty()) {
    takeoff_subscriber_ = ros_node_->create_subscription<std_msgs::msg::Empty>(
      takeoff_topic_, qos,
      std::bind(&DroneSimpleControllerPrivate::TakeoffCallback, this, std::placeholders::_1));
  } else {
    RCLCPP_ERROR(ros_node_->get_logger(), "No takeoff topic defined!");
  }

  // subscribe command: land command
  if (!land_topic_.empty()) {
    land_subscriber_ = ros_node_->create_subscription<std_msgs::msg::Empty>(
      land_topic_, qos,
      std::bind(&DroneSimpleControllerPrivate::LandCallback, this, std::placeholders::_1));
  } else {
    RCLCPP_ERROR(ros_node_->get_logger(), "No land topic defined!");
  }

  // subscribe command: reset command
  if (!reset_topic_.empty()) {
    reset_subscriber_ = ros_node_->create_subscription<std_msgs::msg::Empty>(
      reset_topic_, qos,
      std::bind(&DroneSimpleControllerPrivate::ResetCallback, this, std::placeholders::_1));
  } else {
    RCLCPP_ERROR(ros_node_->get_logger(), "No reset topic defined!");
  }

  // Subscribe command: switch mode command
  if (!switch_mode_topic_.empty()) {
    switch_mode_subscriber_ = ros_node_->create_subscription<std_msgs::msg::Bool>(
      switch_mode_topic_, qos,
      std::bind(&DroneSimpleControllerPrivate::SwitchModeCallback, this, std::placeholders::_1));
  } else {
    RCLCPP_ERROR(ros_node_->get_logger(), "No switch mode topic defined!");
  }
}

void DroneSimpleControllerPrivate::InitPublishers(
  std::string gt_topic_, std::string gt_vel_topic_,
  std::string gt_acc_topic_,
  std::string cmd_mode_topic_,
  std::string state_topic_,
  std::string odom_topic_)
{
  if (!gt_topic_.empty()) {
    pub_gt_pose_ = ros_node_->create_publisher<geometry_msgs::msg::Pose>(gt_topic_, 1024);
  } else {
    RCLCPP_ERROR(ros_node_->get_logger(), "No ground truth topic defined!");
  }

  if (!gt_vel_topic_.empty()) {
    pub_gt_vec_ = ros_node_->create_publisher<geometry_msgs::msg::Twist>("gt_vel", 1024);
  } else {
    RCLCPP_ERROR(ros_node_->get_logger(), "No ground truth velocity topic defined!");
  }

  if (!gt_acc_topic_.empty()) {
    pub_gt_acc_ = ros_node_->create_publisher<geometry_msgs::msg::Twist>("gt_acc", 1024);
  } else {
    RCLCPP_ERROR(ros_node_->get_logger(), "No ground truth acceleration topic defined!");
  }

  if (!cmd_mode_topic_.empty()) {
    pub_cmd_mode = ros_node_->create_publisher<std_msgs::msg::String>(cmd_mode_topic_, 1024);
  } else {
    RCLCPP_ERROR(ros_node_->get_logger(), "No command mode topic defined!");
  }

  if (!state_topic_.empty()) {
    pub_state = ros_node_->create_publisher<std_msgs::msg::Int8>(state_topic_, 1024);
  } else {
    RCLCPP_ERROR(ros_node_->get_logger(), "No state topic defined!");
  }

  if (!odom_topic_.empty()) {
    pub_odom_ = ros_node_->create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 1024);
  } else {
    RCLCPP_ERROR(ros_node_->get_logger(), "No odom topic defined!");
  }
  pub_payload_pos_ = ros_node_->create_publisher<geometry_msgs::msg::Point>("payload_position", 10);
}



// Controller configuration
/**
* @brief Initiliaze the PID params
*
* @param _model shared pointer to the model object
* @param _sdf shared pointer to the sdf object
*/
void DroneSimpleControllerPrivate::LoadControllerSettings(
  gazebo::physics::ModelPtr _model,
  sdf::ElementPtr _sdf)
{
  controllers_.roll.Load(_sdf, "rollpitch");
  controllers_.pitch.Load(_sdf, "rollpitch");
  controllers_.yaw.Load(_sdf, "yaw");
  controllers_.velocity_x.Load(_sdf, "velocityXY");
  controllers_.velocity_y.Load(_sdf, "velocityXY");
  controllers_.velocity_z.Load(_sdf, "velocityZ");

  controllers_.pos_x.Load(_sdf, "positionXY");
  controllers_.pos_y.Load(_sdf, "positionXY");
  controllers_.pos_z.Load(_sdf, "positionZ");

  RCLCPP_INFO_STREAM(
    ros_node_->get_logger(), "Using the PID parameters: \n" <<
      "\tRoll Pitch:\n" << "\t\tkP: " << controllers_.roll.gain_p << ", kI: " << controllers_.roll.gain_i << ",kD: " << controllers_.roll.gain_d << ", Limit: " << controllers_.roll.limit << ", Time Constant: " << controllers_.roll.time_constant << "\n" <<
      "\tYaw:\n" << "\t\tkP: " << controllers_.yaw.gain_p << ", kI: " << controllers_.yaw.gain_i << ",kD: " << controllers_.yaw.gain_d << ", Limit: " << controllers_.yaw.limit << ", Time Constant: " << controllers_.yaw.time_constant << "\n" <<
      "\tVelocity X:\n" << "\t\tkP: " << controllers_.velocity_x.gain_p << ", kI: " << controllers_.velocity_x.gain_i << ",kD: " << controllers_.velocity_x.gain_d << ", Limit: " << controllers_.velocity_x.limit << ", Time Constant: " << controllers_.velocity_x.time_constant << "\n" <<
      "\tVelocity Y:\n" << "\t\tkP: " << controllers_.velocity_y.gain_p << ", kI: " << controllers_.velocity_y.gain_i << ",kD: " << controllers_.velocity_y.gain_d << ", Limit: " << controllers_.velocity_y.limit << ", Time Constant: " << controllers_.velocity_y.time_constant << "\n" <<
      "\tVelocity Z:\n" << "\t\tkP: " << controllers_.velocity_z.gain_p << ", kI: " << controllers_.velocity_z.gain_i << ",kD: " << controllers_.velocity_z.gain_d << ", Limit: " << controllers_.velocity_z.limit << ", Time Constant: " << controllers_.velocity_z.time_constant << "\n" <<
      "\tPosition XY:\n" << "\t\tkP: " << controllers_.pos_x.gain_p << ", kI: " << controllers_.pos_x.gain_i << ",kD: " << controllers_.pos_x.gain_d << ", Limit: " << controllers_.pos_x.limit << ", Time Constant: " << controllers_.pos_x.time_constant << "\n" <<
      "\tPosition Z:\n" << "\t\tkP: " << controllers_.pos_z.gain_p << ", kI: " << controllers_.pos_z.gain_i << ",kD: " << controllers_.pos_z.gain_d << ", Limit: " << controllers_.pos_z.limit << ", Time Constant: " << controllers_.pos_z.time_constant
  );
}

// Callbacks
/**
* @brief Callback function for the drone command topic.
* This function is called whenever a new message is received on the drone command topic. It updates
* the cmd_val member variable with the new command message. It also generates motion noise for the
* drone's angular and linear velocities by adding drift and small noise values to the command message.
* The amount of noise added is determined by the motion_drift_noise_time_ and motion_small_noise_
* member variables of the DroneSimpleController class.
* The function uses the world->SimTime() function to get the current simulator time and calculate the
* time difference between the current and last simulation time. The time difference is used to update
* the drift noise values if the time_counter_for_drift_noise is greater than motion_drift_noise_time_.
* The updated command message is then used to update the cmd_val member variable.
*
* @param cmd Pointer to the command message containing linear and angular velocities.
*/
void DroneSimpleControllerPrivate::CmdCallback(const geometry_msgs::msg::Twist::SharedPtr cmd)
{
  cmd_vel = *cmd;

  static gazebo::common::Time last_sim_time = world->SimTime();
  static double time_counter_for_drift_noise = 0;
  static double drift_noise[4] = {0.0, 0.0, 0.0, 0.0};
  // Get simulator time
  gazebo::common::Time cur_sim_time = world->SimTime();
  double dt = (cur_sim_time - last_sim_time).Double();
  // save last time stamp
  last_sim_time = cur_sim_time;

  // generate noise
  if (time_counter_for_drift_noise > motion_drift_noise_time_) {
    drift_noise[0] = 2 * motion_drift_noise_ * (drand48() - 0.5);
    drift_noise[1] = 2 * motion_drift_noise_ * (drand48() - 0.5);
    drift_noise[2] = 2 * motion_drift_noise_ * (drand48() - 0.5);
    drift_noise[3] = 2 * motion_drift_noise_ * (drand48() - 0.5);
    time_counter_for_drift_noise = 0.0;
  }
  time_counter_for_drift_noise += dt;

  cmd_vel.angular.x += drift_noise[0] + 2 * motion_small_noise_ * (drand48() - 0.5);
  cmd_vel.angular.y += drift_noise[1] + 2 * motion_small_noise_ * (drand48() - 0.5);
  cmd_vel.angular.z += drift_noise[3] + 2 * motion_small_noise_ * (drand48() - 0.5);
  cmd_vel.linear.z += drift_noise[2] + 2 * motion_small_noise_ * (drand48() - 0.5);
}

/**
* @brief Callback function for position control command.
* This function is called when a new position control command is received.
* It sets the m_posCtrl flag to the value of the command data.
* @param cmd The position control command message.
*/
void DroneSimpleControllerPrivate::PosCtrlCallback(const std_msgs::msg::Bool::SharedPtr cmd)
{
  m_posCtrl = cmd->data;
}

/**
* @brief Callback function to handle IMU sensor data.
* @param imu Shared pointer to IMU sensor data.
* The function reads the quaternion data from the IMU sensor and updates the orientation and angular velocity of the drone.
*/
void DroneSimpleControllerPrivate::ImuCallback(const sensor_msgs::msg::Imu::SharedPtr imu)
{
  //directly read the quternion from the IMU data
  pose.Rot().Set(imu->orientation.w, imu->orientation.x, imu->orientation.y, imu->orientation.z);
  euler = pose.Rot().Euler();
  angular_velocity =
    pose.Rot().RotateVector(
    ignition::math::v6::Vector3<double>(
      imu->angular_velocity.x,
      imu->angular_velocity.y, imu->angular_velocity.z));
}

/**
* @brief Callback function to initiate taking off of the drone.
* @param msg Empty message.
*/
void DroneSimpleControllerPrivate::TakeoffCallback(const std_msgs::msg::Empty::SharedPtr msg)
{
  if (navi_state == LANDED_MODEL) {
    navi_state = TAKINGOFF_MODEL;
    m_timeAfterCmd = 0;
    RCLCPP_INFO(ros_node_->get_logger(), "Quadrotor takes off!!");
  }
}

/**
* @brief Callback function to initiate landing of the drone.
* @param msg Empty message.
*/
void DroneSimpleControllerPrivate::LandCallback(const std_msgs::msg::Empty::SharedPtr msg)
{
  if (navi_state == FLYING_MODEL || navi_state == TAKINGOFF_MODEL) {
    navi_state = LANDING_MODEL;
    m_timeAfterCmd = 0;
    RCLCPP_INFO(ros_node_->get_logger(), "Quadrotor lands!!");
  }
}

/**
* @brief Callback function for reset command
* This function resets the controller and the drone's state.
* @param msg Empty message
*/
void DroneSimpleControllerPrivate::ResetCallback(const std_msgs::msg::Empty::SharedPtr msg)
{
  RCLCPP_INFO(ros_node_->get_logger(), "Reset quadrotor!!");
  Reset();
}

/**
* @brief Callback function for receiving a message to switch between velocity and position control modes
* @param msg Shared pointer to the message containing the boolean value for switching the mode
* The function switches between velocity and position control modes based on the boolean value in the message.
* If the boolean value is true, the control mode is switched to velocity control and if it's false, the control
* mode is switched to position control. It also resets the integral term of the controllers for the new mode.
*/
void DroneSimpleControllerPrivate::SwitchModeCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  m_velMode = msg->data;

  std_msgs::msg::String mode;
  if (m_velMode) {
    mode.data = "velocity";
  } else {
    mode.data = "position";
  }
  pub_cmd_mode->publish(mode);
}


void DroneSimpleControllerPrivate::PublishOdom(
  const ignition::math::v6::Pose3<double> & pose,
  const ignition::math::v6::Vector3<double> & velocity,
  const ignition::math::v6::Vector3<double> & acceleration)
{
  // Prepare the Odometry message
  nav_msgs::msg::Odometry odom;
  odom.header.stamp =  gazebo_ros::Convert<builtin_interfaces::msg::Time>(current_time);
  std::string ns = ros_node_->get_namespace();
  odom.header.frame_id = ns + "/odom";
  odom.child_frame_id = ns + "/base_footprint";

  // Set position and orientation from the drone's current state in UpdateDynamics
  odom.pose.pose.position = gazebo_ros::Convert<geometry_msgs::msg::Point>(pose.Pos());
  odom.pose.pose.orientation = gazebo_ros::Convert<geometry_msgs::msg::Quaternion>(pose.Rot());

  // Set velocity information from UpdateDynamics
  auto linear = model->WorldLinearVel();
  auto angular = model->WorldAngularVel();
  
  // Convert velocities to child frame (aka base_footprint)
  auto pose_rot = pose.Rot();
  auto pose_rot_inv = pose_rot.Inverse();
  auto linear_child = pose_rot_inv.RotateVector(linear);
  auto angular_child = pose_rot_inv.RotateVector(angular);
  odom.twist.twist.linear = gazebo_ros::Convert<geometry_msgs::msg::Vector3>(linear_child);
  odom.twist.twist.angular = gazebo_ros::Convert<geometry_msgs::msg::Vector3>(angular_child);
  // Publish the odometry message
  pub_odom_->publish(odom);

  // Publish the TF transformation
  geometry_msgs::msg::TransformStamped transformStamped;
  transformStamped.header.stamp = odom.header.stamp;
  transformStamped.header.frame_id = ns + "/odom";
  transformStamped.child_frame_id = ns + "/base_footprint";
  transformStamped.transform.translation.x = odom.pose.pose.position.x;
  transformStamped.transform.translation.y = odom.pose.pose.position.y;
  transformStamped.transform.translation.z = odom.pose.pose.position.z;
  transformStamped.transform.rotation = odom.pose.pose.orientation;

  tf_broadcaster_->sendTransform(transformStamped);
}


/**
* @brief Updates the current state of the drone.
* This method is responsible for updating the current state of the drone based on the navigation state.
* If the drone is taking off, it checks if the time after the command is greater than 0.5 seconds.
* If it is, it sets the navigation state to flying. If the drone is landing, it checks if the time after the command is greater than 1 second.
* If it is, it sets the navigation state to landed. If the drone is neither taking off nor landing, it resets the time after the command to zero.
*
* @param dt The time elapsed since the last update, in seconds.
*/
void DroneSimpleControllerPrivate::UpdateState(double dt)
{
  if (navi_state == TAKINGOFF_MODEL) {
    m_timeAfterCmd += dt;
    if (m_timeAfterCmd > 0.5) {
      navi_state = FLYING_MODEL;
      std::cout << "Entering flying model!" << std::endl;
    }
  } else if (navi_state == LANDING_MODEL) {
    m_timeAfterCmd += dt;
    if (m_timeAfterCmd > 1.0) {
      navi_state = LANDED_MODEL;
      std::cout << "Landed!" << std::endl;
    }
  } else {
    m_timeAfterCmd = 0;
  }

  // publish current state using pub_state
  std_msgs::msg::Int8 state_msg;
  state_msg.data = navi_state;
  pub_state->publish(state_msg);
}


/**
* @brief Update the dynamics of the drone.
* This method updates the dynamics of the drone based on its current state and the current
* commands being received. It computes the force and torque to be applied to the drone, and
* updates its position, velocity, and orientation accordingly. It also publishes the ground
* truth pose, velocity, and acceleration of the drone to ROS topics.
*
* @param dt The time step to use for the update.
*/
void DroneSimpleControllerPrivate::UpdateDynamics(double dt)
{
  ignition::math::v6::Vector3<double> force, torque;

  // Get Pose/Orientation from Gazebo
  pose = link->WorldPose();
  angular_velocity = link->WorldAngularVel();
  euler = pose.Rot().Euler();
  
  // 保存上一时刻速度用于加速度计算
  static ignition::math::v6::Vector3<double> prev_velocity = link->WorldLinearVel();
  velocity = link->WorldLinearVel();
  
  // 计算总加速度（包含重力）
  acceleration = (velocity - prev_velocity) / dt;
  prev_velocity = velocity;
  
  // 计算净加速度（扣除重力）
  ignition::math::v6::Vector3<double> gravity_world = world->Gravity();
  ignition::math::v6::Vector3<double> net_acceleration = acceleration - gravity_world;

  // === read payload link state ===
  auto payload_link = model->GetLink("end_sphere");
  ignition::math::v6::Pose3<double> payload_pose;
  if (payload_link) {
      payload_pose = payload_link->WorldPose();
      auto payload_vel  = payload_link->WorldLinearVel();

      payload_position = payload_pose.Pos();
      payload_velocity = payload_vel;

      if (pub_payload_pos_) {
          geometry_msgs::msg::Point p_msg;
          p_msg.x = payload_position.X();
          p_msg.y = payload_position.Y();
          p_msg.z = payload_position.Z();
          pub_payload_pos_->publish(p_msg);
      }

      // 计算 cable unit direction (从无人机指向负载)
      auto cable_vector = (payload_position - pose.Pos());
      const double cable_length = cable_vector.Length();
      q = Eigen::Vector3d(cable_vector.X(), cable_vector.Y(), cable_vector.Z()).normalized();

      // 计算 q_dot
      if (cable_length > 1e-5) {
          auto q_dot_numerator = payload_velocity - velocity;
          q_dot = Eigen::Vector3d(q_dot_numerator.X(), q_dot_numerator.Y(), q_dot_numerator.Z()) / cable_length
          - (cable_vector.Dot(q_dot_numerator) / pow(cable_length, 3)) * Eigen::Vector3d(cable_vector.X(), cable_vector.Y(), cable_vector.Z());
      } else {
          // 处理电缆长度接近零的情况
          q = Eigen::Vector3d(0, 0, -1);  // 默认向下
          q_dot = Eigen::Vector3d(0, 0, 0);
      }
  } else {
      RCLCPP_WARN(ros_node_->get_logger(), "Payload link not found! Check URDF for end_sphere.");
  }

  // === define parameters ===
  double mD = mass; // drone mass
  double mL = payload_link ? payload_link->GetInertial()->Mass() : 2.0;  // 默认2kg
  double l  = 0.6;  // 电缆长度
  double g = 9.81;  // 重力加速度
  
  // === 使用净加速度构建动力学方程 ===
  Eigen::Vector3d xDD_net(
      net_acceleration.X(),
      net_acceleration.Y(),
      net_acceleration.Z()
  );

  // 构建质量矩阵
  Eigen::Matrix3d I3 = Eigen::Matrix3d::Identity();
  Eigen::MatrixXd M(6,6);
  M.block<3,3>(0,0) = (mD + mL) * I3;
  M.block<3,3>(0,3) = mL * l * I3;
  M.block<3,3>(3,0) = mL * l * (I3 - q*q.transpose());
  M.block<3,3>(3,3) = mL * l*l * I3;

  // 构建右侧项
  Eigen::Vector3d e3(0,0,1);  // 向上单位向量
  Eigen::Vector3d f_vec(force.X(), force.Y(), force.Z());
  
  // 使用净加速度构建右侧项
  Eigen::Vector3d rhs1 = f_vec;
  Eigen::Vector3d rhs2 = - mL * l*l * q_dot.squaredNorm() * q
                        - mL * g * l * (I3 - q*q.transpose()) * e3;

  Eigen::VectorXd rhs(6);
  rhs.segment<3>(0) = rhs1;
  rhs.segment<3>(3) = rhs2;

  // 求解加速度
  Eigen::VectorXd sol = M.inverse() * rhs;
  Eigen::Vector3d xDD_net_sol = sol.segment<3>(0);
  Eigen::Vector3d qDD_sol = sol.segment<3>(3);

  // === 计算张力 ===
  // 使用物理正确的张力公式
  double T = mL * (
      g * std::abs(q.z())          // 重力分量（始终为正）
      - q.dot(xDD_net_sol)         // 无人机净加速度
      - l * q.dot(qDD_sol)         // 负载旋转加速度
  );
  
  // 确保张力不为负
  T = std::max(T, 0.1 * mL * g);  // 至少为理论值的10%
  
  // double T_gravity_only = mL * g;
  // RCLCPP_INFO_STREAM(ros_node_->get_logger(),
  // "Tension: T = " << T << ", T_gravity_only = " << T_gravity_only);
  // RCLCPP_INFO_STREAM(ros_node_->get_logger(), "q: " << q.transpose());
  // RCLCPP_INFO_STREAM(ros_node_->get_logger(), "xDD_net: " << xDD_net_sol.transpose());
  // RCLCPP_INFO_STREAM(ros_node_->get_logger(), "qDD: " << qDD_sol.transpose());
  
  // === Payload rotational dynamics ===
  if (payload_link) {
      // 获取负载角速度
      ignition::math::v6::Vector3<double> payload_angular_velocity = payload_link->WorldAngularVel();
      Eigen::Vector3d Omega_L(
          payload_angular_velocity.X(),
          payload_angular_velocity.Y(),
          payload_angular_velocity.Z());

      // 负载转动惯量（球体）
      const double payload_radius = 0.1;  // 假设负载半径0.1m
      const double JL_val = 0.4 * mL * payload_radius * payload_radius;  // 球体转动惯量
      Eigen::Matrix3d JL = Eigen::Matrix3d::Identity() * JL_val;

      // 负载方向
      Eigen::Quaterniond qL(
          payload_pose.Rot().W(),
          payload_pose.Rot().X(),
          payload_pose.Rot().Y(),
          payload_pose.Rot().Z());
      Eigen::Matrix3d RL = qL.toRotationMatrix();

      // 负载连接点位置（从质心指向连接点）
      Eigen::Vector3d rc(0, 0, 0);  // 假设连接点在质心

      // 计算扭矩（张力方向为q，从无人机指向负载）
      Eigen::Vector3d tension_vector = T * q;
      Eigen::Vector3d tau_payload = RL * rc.cross(tension_vector);

      // 负载旋转动力学
      Eigen::Vector3d Omega_L_dot =
          JL.inverse() * (tau_payload - Omega_L.cross(JL * Omega_L));

      // 应用角速度变化
      static Eigen::Vector3d Omega_L_accumulated(0,0,0);
      Omega_L_accumulated += Omega_L_dot * dt;
      
      ignition::math::v6::Vector3<double> omega_ign(
          Omega_L_accumulated(0), 
          Omega_L_accumulated(1), 
          Omega_L_accumulated(2));
      
      payload_link->SetAngularVel(omega_ign);
  }

  // === 发布状态信息 ===
  // 发布无人机位姿
  geometry_msgs::msg::Pose gt_pose;
  gt_pose.position = gazebo_ros::Convert<geometry_msgs::msg::Point>(pose.Pos());
  gt_pose.orientation = gazebo_ros::Convert<geometry_msgs::msg::Quaternion>(pose.Rot());
  pub_gt_pose_->publish(gt_pose);

  // 发布速度（机体坐标系）
  ignition::math::v6::Vector3<double> body_vel = pose.Rot().RotateVector(velocity);
  geometry_msgs::msg::Twist vel_msg;
  vel_msg.linear = gazebo_ros::Convert<geometry_msgs::msg::Vector3>(body_vel);
  pub_gt_vec_->publish(vel_msg);

  // 发布加速度（机体坐标系）
  ignition::math::v6::Vector3<double> body_acc = pose.Rot().RotateVector(acceleration);
  geometry_msgs::msg::Twist acc_msg;
  acc_msg.linear = gazebo_ros::Convert<geometry_msgs::msg::Vector3>(body_acc);
  pub_gt_acc_->publish(acc_msg);

  // 更新位置变化
  position = pose.Pos();

  // === 控制器计算 ===
  // 获取重力相关信息
  ignition::math::v6::Vector3<double> gravity_body = pose.Rot().RotateVector(gravity_world);
  double gravity = gravity_body.Length();
  double load_factor = gravity * gravity / gravity_world.Dot(gravity_body);

  // 转换到航向坐标系
  ignition::math::v6::Quaternion<double> heading_quaternion(cos(euler[2] / 2), 0.0, 0.0, sin(euler[2] / 2));
  ignition::math::v6::Vector3<double> velocity_xy = heading_quaternion.RotateVectorReverse(velocity);
  ignition::math::v6::Vector3<double> acceleration_xy = heading_quaternion.RotateVectorReverse(acceleration);
  ignition::math::v6::Vector3<double> angular_velocity_body = pose.Rot().RotateVectorReverse(angular_velocity);

  // 重置力/力矩
  force.Set(0.0, 0.0, 0.0);
  torque.Set(0.0, 0.0, 0.0);

  // === 控制器实现 ===
  if (m_posCtrl) {
    // 位置控制模式
    if (navi_state == FLYING_MODEL) {
      double vx = controllers_.pos_x.update(cmd_vel.linear.x, position[0], position[0] - prev_position.X(), dt);
      double vy = controllers_.pos_y.update(cmd_vel.linear.y, position[1], position[1] - prev_position.Y(), dt);
      double vz = controllers_.pos_z.update(cmd_vel.linear.z, position[2], position[2] - prev_position.Z(), dt);
      
      ignition::math::v6::Vector3<double> vb = heading_quaternion.RotateVectorReverse(
          ignition::math::v6::Vector3<double>(vx, vy, vz));

      double pitch_command = controllers_.velocity_x.update(
          vb[0], velocity_xy[0], acceleration_xy[0], dt) / gravity;
      
      double roll_command = -controllers_.velocity_y.update(
          vb[1], velocity_xy[1], acceleration_xy[1], dt) / gravity;
      
      torque[0] = inertia[0] * controllers_.roll.update(
          roll_command, euler[0], angular_velocity_body[0], dt);
      
      torque[1] = inertia[1] * controllers_.pitch.update(
          pitch_command, euler[1], angular_velocity_body[1], dt);
      
      // 使用总质量计算升力（无人机+负载）
      force[2] = (mass + mL) * (
          controllers_.velocity_z.update(vz, velocity[2], acceleration[2], dt) 
          + load_factor * gravity);
    }
  } else {
    // 普通控制模式
    if (navi_state == FLYING_MODEL) {
      // 悬停控制
      double pitch_command = controllers_.velocity_x.update(
          cmd_vel.linear.x, velocity_xy[0], acceleration_xy[0], dt) / gravity;
      
      double roll_command = -controllers_.velocity_y.update(
          cmd_vel.linear.y, velocity_xy[1], acceleration_xy[1], dt) / gravity;
      
      torque[0] = inertia[0] * controllers_.roll.update(
          roll_command, euler[0], angular_velocity_body[0], dt);
      
      torque[1] = inertia[1] * controllers_.pitch.update(
          pitch_command, euler[1], angular_velocity_body[1], dt);
    } else {
      // 着陆控制
      if (m_velMode) {
        double pitch_command = controllers_.velocity_x.update(
            cmd_vel.angular.x, velocity_xy[0], acceleration_xy[0], dt) / gravity;
        
        double roll_command = -controllers_.velocity_y.update(
            cmd_vel.angular.y, velocity_xy[1], acceleration_xy[1], dt) / gravity;
        
        torque[0] = inertia[0] * controllers_.roll.update(
            roll_command, euler[0], angular_velocity_body[0], dt);
        
        torque[1] = inertia[1] * controllers_.pitch.update(
            pitch_command, euler[1], angular_velocity_body[1], dt);
      } else {
        // 倾斜控制
        torque[0] = inertia[0] * controllers_.roll.update(
            cmd_vel.angular.x, euler[0], angular_velocity_body[0], dt);
        
        torque[1] = inertia[1] * controllers_.pitch.update(
            cmd_vel.angular.y, euler[1], angular_velocity_body[1], dt);
      }
    }
    
    torque[2] = inertia[2] * controllers_.yaw.update(
        cmd_vel.angular.z, angular_velocity[2], 0, dt);
    
    // 使用总质量计算升力（无人机+负载）
    force[2] = (mass + mL) * (
        controllers_.velocity_z.update(
            cmd_vel.linear.z, velocity[2], acceleration[2], dt) 
        + load_factor * gravity);
  }

  // 限制升力范围
  if (max_force_ > 0.0 && force[2] > max_force_) {
    force[2] = max_force_;
  }
  if (force[2] < 0.0) {
    force[2] = 0.0;
  }

  // 应用力和力矩
  if (navi_state == FLYING_MODEL) {
    link->AddRelativeForce(force);
    link->AddRelativeTorque(torque);
  } else if (navi_state == TAKINGOFF_MODEL) {
    link->AddRelativeForce(force * 1.5);
    link->AddRelativeTorque(torque * 1.5);
  } else if (navi_state == LANDING_MODEL) {
    link->AddRelativeForce(force * 0.8);
    link->AddRelativeTorque(torque * 0.8);
  }

  // 发布里程计
  if (pub_odom) {
    if (current_time.Double() - last_odom_publish_time_ >= 1.0 / odom_hz) {
      PublishOdom(pose, velocity, acceleration);
      last_odom_publish_time_ = current_time.Double();
      odom_seq++;
    }
  }
  
  // 保存当前位置用于下一帧计算
  prev_position = position;
}

} // namespace gazebo_plugins