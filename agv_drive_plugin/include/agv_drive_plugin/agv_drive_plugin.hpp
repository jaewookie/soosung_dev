#ifndef AGV_DRIVE_PLUGIN_HPP_
#define AGV_DRIVE_PLUGIN_HPP_

#include <gazebo/common/Plugin.hh>
#include <memory>

namespace gazebo_ros
{

class AgvDrivePluginPrivate;

class AgvDrivePlugin : public gazebo::ModelPlugin
{
public:
  /// Constructor:
  AgvDrivePlugin();

  /// Destructor:
  virtual ~AgvDrivePlugin();

  // LOAD plugin:
  void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf) override;

private:

  std::unique_ptr<AgvDrivePluginPrivate> impl_;
};

}  // namespace gazebo_ros

#endif  // AGV_DRIVE_PLUGIN_HPP_
