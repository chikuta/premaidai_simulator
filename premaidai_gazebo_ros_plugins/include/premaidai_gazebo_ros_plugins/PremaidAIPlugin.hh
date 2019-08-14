#ifndef GAZEBO_PREMAIDAI_PLUGIN_HH
#define GAZEBO_PREMAIDAI_PLUGIN_HH

#include <string>
#include <vector>
#include <map>

// include boost headers
#include <boost/thread.hpp>

// include ros headers
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

// gazebo libraries
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>

namespace gazebo
{
  class PremaidAIPlugin : public ModelPlugin
  {
  public:
    PremaidAIPlugin();
    virtual ~PremaidAIPlugin();
    virtual void Load(physics::ModelPtr parent, sdf::ElementPtr sdf);
    virtual void OnUpdate();

  protected:
    void JointCommandCallback(const sensor_msgs::JointState::ConstPtr& data);

  private:
    // typedef
    typedef std::vector<std::string> StringArray;
    typedef std::vector<sensor_msgs::JointState> JointCommandQueue;
    typedef std::map<std::string, physics::JointPtr> JointPtrMap;

    event::ConnectionPtr update_connection_;        /** 更新通知 */
    physics::ModelPtr model_;                       /** モデル情報にアクセスするために使用 */
    physics::JointControllerPtr joint_controller_;   /** */
    StringArray joint_names_;                       /** 何度も使用するのでメンバ変数として定義 */
    JointPtrMap joint_map_;

    // ros related variables
    boost::mutex joint_command_mutex_;        /** mutex */
    JointCommandQueue joint_command_queue_;   /** ROS tocpic にて取得したデータを一時保存するために使用 */
    sensor_msgs::JointState joint_command_;   /** */
    ros::Subscriber joint_command_sub_;       /** 操作コマンド取得用 */
    ros::Publisher joint_states_pub_;         /**  */
    ros::NodeHandlePtr node_handle_;          /** ノードハンドラ */
  };
}


#endif // GAZEBO_PREMAIDAI_PLUGIN_HH
