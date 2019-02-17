# Remote robot

[![Python](https://img.shields.io/badge/python-3.6-blue.svg?style=flat-square)](https://docs.python.org/3/)
[![Python](https://img.shields.io/badge/python-2.7-blue.svg?style=flat-square)](https://docs.python.org/2/)
[![Django](https://img.shields.io/badge/django-2.1-blue.svg?style=flat-square)](https://www.djangoproject.com)
[![ROS](https://img.shields.io/badge/ROS-kinetic-blue.svg)](http://wiki.ros.org/kinetic)

**Control your robot remotely!**

## Contents

- [Install](#install)
- [User guide](#user-guide)
- [Features](#features)
- [Techniques](#techniques)
- [Test commands](#test-commands)
- [Contributing](#contributing)


## Install

You'll need:
- Python 3.6+ for the webserver
- Python 2.7 for ROS
- Install [ROS kinetic](http://wiki.ros.org/kinetic)

ROS runs on GNU/Linux, but other systems may be supported (more info [here](http://wiki.ros.org/kinetic)). We recommend using the Ubuntu 16.04 (Xenial) release.


To download the project, you can use git
```bash
git clone https://github.com/AB036/remote_robot
```
or download it from Github.


### Server

#### Web server
First, prepare the environment of the web server:

```bash
cd remote_robot_web
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
python3 manage.py migrate
```

#### ROS server

Now let's prepare the ROS part of the server. Open a new terminal and prepare the ROS environment:
```bash
mkdir -p ~/catkin_ws/src
cd catkin_ws/
catkin_make
source devel/setup.bash
```
The path of your catkin workspace might change depending on your ROS installation.


Put the content of `remote_robot_ros/` into  `catkin_ws/src/` and install the libraries:
```bash
cd catkin_ws/src/remote_robot/
pip install -r requirements.txt
```

Then make the file `ros_server.py` executable with:
```bash
cd catkin_ws/src/remote_robot/scripts/
chmod +x ros_server.py
```

Launch catkin_make from your catkin workspace.
```bash
cd ~/catkin_ws
catkin_make
```


## User guide

### Server

#### Web server

Open a new terminal and cd into the project. Start the development server:

```bash
cd remote_robot_web
python3 manage.py runserver --noreload
```
The app will be available at http://127.0.0.1:8000.


#### ROS server

Open a new terminal and start the ROS core:
```bash
source ~/catkin_ws/devel/setup.bash
roscore
```
Once again, the path of your catkin workspace might change depending on your ROS installation.


Now open a new terminal and start the ROS node:
```bash
source ~/catkin_ws/devel/setup.bash
rosrun remote_robot ros_server.py
```

### The robot
#### Connecting the robot
Just connect your robot to the server by adding the server to the file `/etc/hosts` and configuring the following environment variable:
```bash
export ROS_MASTER_URI=http://<ip_server>:<port>
```

Now you can start:
- Subscribing to commands on the node `commands`
- Publishing the video stream on the node `video_frame`.

#### Using our package (Raspberry - Python 2.7)
If you wish to use our package for the robot, you'll need a Raspberry with Python 2.7.

Prepare the ROS environment:
```bash
mkdir -p ./catkin_ws/src
cd catkin_ws/
catkin_make
source devel/setup.bash
```

Put the content of `rasp/Code` in  `catkin_ws/src/giopek/` and install the libraries:
```bash
cd catkin_ws/src/giopek/
pip install -r requirements.txt
```

Then make the file `ros_package_raspberry.py` executable with:
```bash
cd catkin_ws/src/giopek/scripts/
chmod +x ros_package_raspberry.py
```

[Connect to the server](#connecting-the-robot) and launch the node:
```bash
cd catkin_ws/
catkin_make
source devel/setup.bash
rosrun giopek ros_package_raspberry.py
```

### Virtual robot
If you wish to test the project and you don't have a robot, we built a virtual robot that simulates a real one. You just have to make the file executable and run it
```bash
cd catkin_ws/src/remote_robot/scripts/
chmod +x virtualbot.py
cd catkin_ws/
catkin_make
source devel/setup.bash
rosrun ros_remote_robot_server virtualbot.py
```

## Features

Here's what you can do with Remote Robot:

- View safely from your computer what your robot sees.
- Control the robot to explore the environment.
- Chat with your friends about the livestream.

## Techniques

Here are the techniques we used to build Remote Robot:

- Object Oriented Programming (OOP) in Python.
- Client-server architecture.
- Web application development using Django.
- HTML/CSS/JS web development.
- Embedded programming in Python on Raspberry Pi.
- Communication with a robot using ROS framework.
- Real time communication of 2 programs using different version of Python with sockets.
- Collaborative development using GitHub, Pull Requests and Trello.

## Test commands

Testing the command management received by Django and the socket communication between the ROS node and the Django server:
```bash
cd remote_robot_web/
python manage.py test control_board
```

...

## Contributing

- Create a branch, e.g. `feature/new-feature` or `fix/very-nasty-bug`.
- Add commits.
- When ready, push to remote: `git push -u origin feature/new-feature`.
- Open a Pull Request: document the changes and provide any useful additional context.
- Ask someone to review your code.
- When ready: merge it.
