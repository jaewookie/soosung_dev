// #include <gazebo/gazebo.hh>
// #include <gazebo/physics/physics.hh>
// #include <gazebo/physics/Model.hh>
// #include <gazebo/physics/World.hh>
// #include <gazebo/physics/Joint.hh>
// #include <gazebo/physics/Link.hh>
// #include <gazebo_ros/conversions/builtin_interfaces.hpp>
// #include <gazebo_ros/conversions/geometry_msgs.hpp>
// #include <gazebo_ros/node.hpp>
// #include <rclcpp/rclcpp.hpp>

// #include "agv_drive_plugin/lslidar_plugin.hpp"

// namespace gazebo_ros
// {

//   class LSLidarPluginPrivate
//   {
//   public:
//     gazebo_ros::Node::SharedPtr ros_node_;
//   };

//   LSLidarPlugin::LSLidarPlugin()
//       : impl_(std::make_unique<LSLidarPluginPrivate>())
//   {
//   }

//   LSLidarPlugin::~LSLidarPlugin()
//   {
//   }

//   void LSLidarPlugin::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
//   {
//     impl_->ros_node_ = gazebo_ros::Node::Get(_sdf);

//     RCLCPP_INFO(impl_->ros_node_->get_logger(), "attached");
//   }

//   GZ_REGISTER_MODEL_PLUGIN(LSLidarPlugin)
// }
