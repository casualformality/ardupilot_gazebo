#include "../include/ScanseSensorPlugin.hh"

using namespace gazebo;
GZ_REGISTER_SENSOR_PLUGIN(ScanseSensorPlugin)

/////////////////////////////////////////////////
ScanseSensorPlugin::ScanseSensorPlugin() : SensorPlugin()
{
}

/////////////////////////////////////////////////
ScanseSensorPlugin::~ScanseSensorPlugin()
{
}

/////////////////////////////////////////////////
void ScanseSensorPlugin::Load(sensors::SensorPtr _sensor, sdf::ElementPtr /*_sdf*/)
{

    std::cerr << "Loading sensor\n";

    // Get the parent sensor.
    this->parentSensor =
            std::dynamic_pointer_cast<sensors::RaySensor>(_sensor);

    // Make sure the parent sensor is valid.
    if (!this->parentSensor)
    {
        gzerr << "ScanseSensorPlugin requires a RaySensor.\n";
        return;
    }
    
    physics::WorldPtr world = physics::get_world(this->parentSensor->WorldName());
    std::string parentName = this->parentSensor->ParentName();
    this->parentObject = world->EntityByName(parentName);


    // Connect to the sensor update event.
    this->updateConnection = this->parentSensor->ConnectUpdated(
            std::bind(&ScanseSensorPlugin::OnUpdate, this));

    // Make sure the parent sensor is active.
    this->parentSensor->SetActive(true);
}

/////////////////////////////////////////////////
void ScanseSensorPlugin::OnUpdate()
{
    ignition::math::Pose3d pose = 
            this->parentSensor->Pose() + this->parentObject->WorldPose();;
    double range = this->parentSensor->Range(0);
    double angle = pose.Rot().Yaw();
    
    if (angle < 0.1 && angle > -0.1) {
        std::cerr << "Angle: " << angle << ", Range: " << range << "\n";
    }
}
