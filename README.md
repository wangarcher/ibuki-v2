# ibuki-v2
the silva system of ibuki v2 (developing)

master branch: [![Build Status](https://travis-ci.org/ustyui/ibuki-v2.svg?branch=master)](https://travis-ci.org/ustyui/ibuki-v2)

## Usage
Can be used to control joints of ibuki.
## Dependencies
- [ROS](http://www.ros.org)
- Catkin workspace

## Building 
```
cd ~/some_path
git clone https://github.com/ustyui/ibuki-v2.git
cd ~/catkin_ws
ln -s ~/some_path/ibuki-v2 ~/catkin_ws/src/silva_beta
catkin_make
```
## How to Use
### System Boot
```
roslaunch silva_beta beta.launch
```

### GUI Usage
```
rosrun silva_beta debug_gui.py
```

### Humanoid Simulation Model Mapping 
see docs in /doc/data_input_format.csv

To run the example, run
```
rosrun silva_beta HSM_csv.py lookaround
```
You can monitor the output using
```
rostopic echo /silva/slave_local/operation
```
### Messages
Silva has different type of messages based on its framework.

You can check the message type by using
```
rostopic list /silva
```
