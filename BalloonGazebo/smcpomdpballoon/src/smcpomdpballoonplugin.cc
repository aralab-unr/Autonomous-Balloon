#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <thread>
#include <memory>

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

class SPBalloonPlugin : public gazebo::ModelPlugin {
public:
  SPBalloonPlugin() : ModelPlugin(), fx(0), fy(0), fh(0) {
    RCLCPP_INFO(rclcpp::get_logger("spballoon_plugin"), "Starting spballoon_plugin");
  }

  void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf) override {
    this->model = _model;

    // Initialize ROS2 node if it hasn't been initialized yet
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }

    // Create ROS2 node
    node_ = std::make_shared<rclcpp::Node>("spballoon_plugin_node");

    // Read parameters from SDF
    this->rate = _sdf->HasElement("updateRate") ? _sdf->Get<double>("updateRate") : 1000.0;
    this->publish_tf = _sdf->HasElement("publishTf") ? _sdf->Get<bool>("publishTf") : true;

    RCLCPP_INFO(node_->get_logger(), "ROS2 Model Plugin Loaded with parameters!");

    // Subscribe to propeller velocity command topic
    sub_ = node_->create_subscription<std_msgs::msg::Float64MultiArray>("/forces", 10, std::bind(&SPBalloonPlugin::ActivateCallback, this, std::placeholders::_1));

    // Start ROS spinning in a separate thread
    ros_thread_ = std::thread([this]() { rclcpp::spin(node_); });

    jointleft = _model->GetJoint("pbl");
    jointright = _model->GetJoint("pbr");
  
    if (!jointleft || !jointright) {
      RCLCPP_ERROR(node_->get_logger(), "Failed to get one or more joints.");
      return;
    }
    // Connect to the world update event
    updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(
      std::bind(&SPBalloonPlugin::OnUpdate, this));
  }

  void OnUpdate() {
    applyForces(fx, fy, fh);
    double kt = 0.00025;
    double propvel = sqrt(fh / kt);
    jointleft->SetVelocity(0, propvel);
    jointright->SetVelocity(0, -propvel);
  }

  void ActivateCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    if (msg->data.size() != 3) {
      RCLCPP_WARN(node_->get_logger(), "Received incorrect number of forces.");
      return;
    } 
    fx = msg->data[0];
    fy = msg->data[1];
    fh = msg->data[2];
  }

private:
  rclcpp::Node::SharedPtr node_;  // ROS2 node handle
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_;
  std::thread ros_thread_;
  double rate;
  bool publish_tf;
  double fx, fy, fh;
  gazebo::physics::ModelPtr model;
  gazebo::physics::JointPtr jointleft;
  gazebo::physics::JointPtr jointright;
  gazebo::event::ConnectionPtr updateConnection;

  void applyForces(double fx, double fy, double fh) {
    // Get links
    gazebo::physics::LinkPtr base = model->GetLink("base_link");
    if (base) {
      base->AddForce(ignition::math::Vector3d(fx, fy, fh));
    } else {
      RCLCPP_WARN(node_->get_logger(), "base_link not found!");
    }
  }
};

GZ_REGISTER_MODEL_PLUGIN(SPBalloonPlugin)