#ifndef _GAZEBO_SCANSE_PLUGIN_HH_
#define _GAZEBO_SCANSE_PLUGIN_HH_

#include <string>

#include <gazebo/gazebo.hh>
#include <gazebo/sensors/sensors.hh>

namespace gazebo
{
  /// \brief An example plugin for a Scanse sensor.
  class ScanseSensorPlugin : public SensorPlugin
  {
    /// \brief Constructor.
    public: ScanseSensorPlugin();

    /// \brief Destructor.
    public: virtual ~ScanseSensorPlugin();

    /// \brief Load the sensor plugin.
    /// \param[in] _sensor Pointer to the sensor that loaded this plugin.
    /// \param[in] _sdf SDF element that describes the plugin.
    public: virtual void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf);

    /// \brief Callback that receives the Scanse sensor's update signal.
    private: virtual void OnUpdate();

    /// \brief Pointer to the Scanse sensor
    private: sensors::RaySensorPtr parentSensor;

    /// \brief Connection that maintains a link between the Scanse sensor's
    /// updated signal and the OnUpdate callback.
    private: event::ConnectionPtr updateConnection;

    /// \brief Pointer to parent for pose estimation
    private: physics::EntityPtr parentObject;
  };
}
#endif
