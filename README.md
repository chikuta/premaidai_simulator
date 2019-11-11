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
  $ wget https://raw.githubusercontent.com/chikuta/premaidai_simulator/master/premaidai_simulator.rosinstall .rosinstall
  $ rosinstall .
  $ catkin build
```

## How to launch

```bash
  $ cd ws
  $ source devel/setup.bash
  $ roslaunch premaidai_gazebo premaidai_empty_world.launch
```

## Referes

* [drcsim](https://bitbucket.org/osrf/drcsim/src/default/)
* [Setting Velocity on Joints and Links](http://gazebosim.org/tutorials?tut=set_velocity)

## LICENSE
MIT
