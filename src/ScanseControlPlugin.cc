#ifndef _SCANSE_PLUGIN_HH_
#define _SCANSE_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

namespace gazebo
{
    /// \brief A plugin to control a Scanse Sweep sensor.
    class ScanseControlPlugin : public ModelPlugin
    {
        /// \brief Constructor
        public: ScanseControlPlugin() {}

        /// \brief The load function is called by Gazebo when the plugin is
        /// inserted into simulation
        /// \param[in] _model A pointer to the model that this plugin is
        /// attached to.
        /// \param[in] _sdf A pointer to the plugin's SDF element.
        public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
        {
            // Just output a message for now
            std::cerr << "\nThe Scanse Sweep plugin is attach to model[" <<
                    _model->GetScopedName() << "]\n";

            //std::dynamic_pointer_cast<physics::ModelPtr>(_model->Get)

            this->model = _model;

            // Store the model and joint pointers for convenience.
            this->joint = this->model->GetJoint("iris_demo::scanse_sweep::scanse_joint");
            
            // Setup and apply P-controller to the joint
            this->pid   = common::PID(0.0001, 0, 0);
            this->model->GetJointController()->SetVelocityPID(
                    this->joint->GetScopedName(), this->pid);

            // Set joint's target velocity. The Scanse Sweep default is 5Hz
            this->model->GetJointController()->SetVelocityTarget(
                    this->joint->GetScopedName(), 31.4);
        }

        /// \brief Pointer to the model.
        private: physics::ModelPtr model;

        /// \brief Pointer to the joint.
        private: physics::JointPtr joint;

        /// \brief A PID controller for the joint.
        private: common::PID pid;
    };

    // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
    GZ_REGISTER_MODEL_PLUGIN(ScanseControlPlugin)
}
#endif
