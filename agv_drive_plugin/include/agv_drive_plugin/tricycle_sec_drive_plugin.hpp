#ifndef TRICYCLE_SEC_DRIVE_PLUGIN_HPP_
#define TRICYCLE_SEC_DRIVE_PLUGIN_HPP_

#include <gazebo/common/Plugin.hh>
#include <memory>

namespace gazebo_ros
{

class TricycleSecDrivePluginPrivate;

class TricycleSecDrivePlugin : public gazebo::ModelPlugin
{
public:
  /// Constructor:
  TricycleSecDrivePlugin();

  /// Destructor:
  virtual ~TricycleSecDrivePlugin();

  // LOAD plugin:
  void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf) override;

  void Reset() override;

private:

  std::unique_ptr<TricycleSecDrivePluginPrivate> impl_;
};

}  // namespace gazebo_ros

#endif  // TRICYCLE_SEC_DRIVE_PLUGIN_HPP_
