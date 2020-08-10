# F1TENTH-RL-BDI

[TODO]

## Quick start
Build the project

	./gradlew build

Stop previous istances

	./gradlew --stop

Launch the f1tenth simulator:
+ Go to the working directory of the simulator (*/simulator*)

`$ source devel/setup.bash`

`$ roslaunch f1tenth_simulator simulator.launch`

Run the python agent server

	./gradlew runPythonAgent


Run the python environment server

	./gradlew runPythonEnv

Run the agent system:

	./gradlew run


## Installation

### Car simulator & RL algorithm

1) Install [ROS Melodic (desktop-full)](http://wiki.ros.org/melodic/Installation/Ubuntu)

2) Install the dependencies

    `$ sudo apt-get install python3-pip python3-yaml`

    `$ pip3 install rospkg catkin_pkg`

    `$ sudo apt-get install ros-melodic-ackermann-msgs`

3) __Optional__ dependencies

    You need to install these packets *only if* you want to use the relative function

    To visualize the images built from lidar data (lidar-to-image = True, show-image = True) you need opencv:

    `$ pip3 install opencv-python`

    To use compression of replay buffer (--compress-replay):

    `$ pip3 install blosc`

4) Setup the car simulator:

    `sudo apt-get install ros-melodic-map-server ros-melodic-joy`

    `$ mkdir -p simulator/src`

    `$ cd simulator/src`

    `$ git clone https://github.com/f1tenth/f1tenth_simulator.git`

    `$ cd ../`

    `$ catkin_make`

5) Install tensorflow 2.1.x

    `$ pip3 install tensorflow`


### Jason-RL framework
1) Install Java (>= 11)

	sudo add-apt-repository ppa:linuxuprising/java
	sudo apt update
	sudo apt-get install default-jre

2) install the python dependencies

	pip3 install flask flask-jsonpify flask-restful

# Configuration
## Algorithm Parameters
You can change the parameters with the agent beliefs.
