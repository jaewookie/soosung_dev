#ifndef TRICYCLE_DRIVE_PLUGIN_HPP_
#define TRICYCLE_DRIVE_PLUGIN_HPP_

#include <gazebo/common/Plugin.hh>
#include <memory>

namespace gazebo_ros
{

class TricycleDrivePluginPrivate;

class TricycleDrivePlugin : public gazebo::ModelPlugin
{
public:
  /// Constructor:
  TricycleDrivePlugin();

  /// Destructor:
  virtual ~TricycleDrivePlugin();

  // LOAD plugin:
  void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf) override;

  void Reset() override;

private:

  std::unique_ptr<TricycleDrivePluginPrivate> impl_;
};

}  // namespace gazebo_ros

#endif  // TRICYCLE_DRIVE_PLUGIN_HPP_
