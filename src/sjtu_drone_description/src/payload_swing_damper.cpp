#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/wrench.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <ignition/math/Vector3.hh>

namespace gazebo
{

class PayloadSwingDamper : public ModelPlugin
{
public:
  PayloadSwingDamper() {}

  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    model_ = model;

    node_ = gazebo_ros::Node::Get(sdf);
    if (!node_)
    {
      gzerr << "[PayloadSwingDamper] Failed to get ROS node from gazebo_ros.\n";
      return;
    }

    std::string link_name = sdf->Get<std::string>("link_name", "end_sphere").first;
    payload_link_ = model_->GetLink(link_name);
    if (!payload_link_)
    {
      gzerr << "[PayloadSwingDamper] Link [" << link_name << "] not found!\n";
      return;
    }

    force_sub_ = node_->create_subscription<geometry_msgs::msg::Wrench>(
      "/payload_force_cmd", 10,
      std::bind(&PayloadSwingDamper::OnForceCmd, this, std::placeholders::_1));

    // === 创建速度发布器 ===
    payload_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(
      "/simple_drone/payload_velocity", 10);

    update_conn_ = event::Events::ConnectWorldUpdateBegin(
      std::bind(&PayloadSwingDamper::OnUpdate, this));

    RCLCPP_INFO(node_->get_logger(), "[PayloadSwingDamper] Plugin loaded successfully.");
  }

  void OnForceCmd(const geometry_msgs::msg::Wrench::SharedPtr msg)
  {
    latest_force_ = *msg;
  }

  void OnUpdate()
  {
    if (!payload_link_)
      return;

    // === 施加力 ===
    ignition::math::Vector3d force(
      latest_force_.force.x,
      latest_force_.force.y,
      latest_force_.force.z);
    payload_link_->AddForce(force);

    // === 发布线速度 ===
    ignition::math::Vector3d vel = payload_link_->WorldLinearVel();
    geometry_msgs::msg::Twist vel_msg;
    vel_msg.linear.x = vel.X();
    vel_msg.linear.y = vel.Y();
    vel_msg.linear.z = vel.Z();
    payload_vel_pub_->publish(vel_msg);
  }

private:
  physics::ModelPtr model_;
  physics::LinkPtr payload_link_;
  event::ConnectionPtr update_conn_;

  gazebo_ros::Node::SharedPtr node_;
  rclcpp::Subscription<geometry_msgs::msg::Wrench>::SharedPtr force_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr payload_vel_pub_;  // ✅ 新增
  geometry_msgs::msg::Wrench latest_force_;
};

GZ_REGISTER_MODEL_PLUGIN(PayloadSwingDamper)

}  // namespace gazebo
