# README

## Environments
* Ubuntu 18.04
* ROS melodic
* gazebo 9.0.0

## How to build

```bash
  $ mkdir -p ws/src
  $ cd ws
  $ catkin init
  $ wget https://raw.githubusercontent.com/chikuta/premaidai_ros_bridge/master/premaidai_simulator.rosinstall .rosinstall
  $ rosinstall .
  $ catkin build
```

## How to launch

### Launch simulator

```bash
  $ cd ws
  $ source devel/setup.bash
  $ roslaunch premaidai_gazebo premaidai_empty_world.launch
```

### Launch simulator with rviz control gui

```bash
  $ cd ws
  $ source devel/setup.bash
  $ roslaunch premaidai_gazebo rviz_control.launch
```

## TODO

- [ ] gazebo plugin 作成
- [ ] simulatorパラメータ調整（現在はDRCSim仕様）

## Referes

* [drcsim](https://bitbucket.org/osrf/drcsim/src/default/)
* [Setting Velocity on Joints and Links](http://gazebosim.org/tutorials?tut=set_velocity)

## LICENSE
MIT