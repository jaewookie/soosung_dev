#ifndef FORK_CONTROL_PLUGIN_HPP_
#define FORK_CONTROL_PLUGIN_HPP_

#include <gazebo/common/Plugin.hh>
#include <memory>

namespace gazebo_ros
{

class ForkControlPluginPrivate;

class ForkControlPlugin : public gazebo::ModelPlugin
{
public:
  /// Constructor:
  ForkControlPlugin();

  /// Destructor:
  virtual ~ForkControlPlugin();

  // LOAD plugin:
  void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf) override;

  void Reset() override;

private:

  std::unique_ptr<ForkControlPluginPrivate> impl_;
};

}  // namespace gazebo_ros

#endif  // FORK_CONTROL_PLUGIN_HPP_
