#include <premaidai_gazebo_ros_plugins/PremaidAIPlugin.hh>

#include <boost/format.hpp>
#include <boost/bind.hpp>
#include <iostream>
#include <stdexcept>

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(PremaidAIPlugin)


PremaidAIPlugin::PremaidAIPlugin()
  : model_()
  , joint_controller_()
  , joint_names_()
  , joint_map_()
  , joint_command_queue_()
  , joint_command_sub_()
  , joint_states_pub_()
  , node_handle_()
{}

// virtual
PremaidAIPlugin::~PremaidAIPlugin()
{
  node_handle_->shutdown();
  joint_command_queue_.clear();
}

// virtual
void PremaidAIPlugin::Load(physics::ModelPtr parent, sdf::ElementPtr sdf)
{
  // initialize model
  model_ = parent;
  joint_controller_ = physics::JointControllerPtr(new physics::JointController(model_));

  // set joint names
  const physics::Joint_V &joints = model_->GetJoints();
  for (unsigned int idx = 0; idx < joints.size(); ++idx)
  {
    std::string name = joints[idx]->GetName();
    joint_names_.push_back(name);
    joint_map_[name] = joints[idx];

    // setup joint controller
    joint_controller_->AddJoint(joints[idx]);
    std::string scoped_name = joints[idx]->GetScopedName();

    // TODO 現在は適当な値
    joint_controller_->SetVelocityPID(scoped_name, common::PID(100, 10, 2));
    joint_controller_->SetPositionPID(scoped_name, common::PID(15, 1, 0));

    // initialize joint_command_
    joint_command_.name.push_back(name);
    joint_command_.position.push_back(0);
    joint_command_.velocity.push_back(0);
    joint_command_.effort.push_back(0);
  }

  // register update function
  this->update_connection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&PremaidAIPlugin::OnUpdate, this));

  // create node handle
  node_handle_ = ros::NodeHandlePtr(new ros::NodeHandle(""));

  // setup topics
  joint_command_sub_ = node_handle_->subscribe<sensor_msgs::JointState>("joint_command", 10, &PremaidAIPlugin::JointCommandCallback, this);
  joint_states_pub_ = node_handle_->advertise<sensor_msgs::JointState>("joint_states", 5);
}

// virtual
void PremaidAIPlugin::OnUpdate()
{
  // get joint state
  if (joint_command_queue_.size() > 0)
  {
    // get joint command data;
    boost::mutex::scoped_lock lock(joint_command_mutex_);
    joint_command_ = joint_command_queue_[0];
    joint_command_queue_.erase(joint_command_queue_.begin());
  }

  for (unsigned int idx = 0; idx < joint_command_.name.size(); ++idx)
  {
    const std::string name = joint_command_.name[idx];
    if (joint_map_.find(name) != joint_map_.end())
    {
      // calc joint velocity
      const std::string scoped_name = joint_map_[name]->GetScopedName();
      joint_controller_->SetPositionTarget(scoped_name, joint_command_.position[idx]);
    }
    else
    {
      ROS_INFO("[%s] joint is not found.", name.c_str());
    }
  }

  // collect joint info for joint_state
  sensor_msgs::JointState joint_state;
  for (JointPtrMap::iterator it = joint_map_.begin(); it != joint_map_.end(); ++it)
  {
    joint_state.name.push_back(it->second->GetName());
    joint_state.position.push_back(it->second->Position(0));
    joint_state.velocity.push_back(it->second->GetVelocity(0));
    joint_state.effort.push_back(it->second->GetForce(0));
  }
  joint_states_pub_.publish(joint_state);

  joint_controller_->Update();
}

void PremaidAIPlugin::JointCommandCallback(const sensor_msgs::JointState::ConstPtr& data)
{
  boost::mutex::scoped_lock lock(joint_command_mutex_);
  joint_command_queue_.push_back(*data);
}