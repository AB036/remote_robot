# Remote robot

[![Python](https://img.shields.io/badge/python-3.6-blue.svg?style=flat-square)](https://docs.python.org/3/)
[![Python](https://img.shields.io/badge/python-2.7-blue.svg?style=flat-square)](https://docs.python.org/2/)
[![Django](https://img.shields.io/badge/django-2.1-blue.svg?style=flat-square)](https://www.djangoproject.com)
[![ROS](https://img.shields.io/badge/ROS-kinetic-blue.svg)](http://wiki.ros.org/kinetic)

**Control your robot remotely!**

## Contents

- [Install](#install)
- [Quick start](#quickstart)
- [Settings](#settings)
- [Features](#features)
- [Techniques](#techniques)
- [Test commands](#test-commands)
- [Contributing](#contributing)


## Install

Currently, it only works with Linux 16.04. You'll need:
- Python 3.6+ for the webserver
- Python 2.7 for ROS
- Install [ROS kinetic](http://wiki.ros.org/kinetic)

## Quickstart

### Server
First, prepare the environment of the web server:

```bash
cd remote_robot_web
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
python manage.py migrate
```

Start the development server:

```bash
python manage.py runserver
```
The app will be available at http://localhost:8000.

Secondly open a new terminal, prepare the ROS environment and start the ROS Master:
```bash
mkdir -p ./catkin_ws/src
mv ros_remote_robot_server/ catkin_ws/src
cd catkin_ws/
catkin_make
source devel/setup.bash
chmod +x src/ros_remote_robot_server/scripts/ros_server.py
catkin_make
roscore
```

Now open a new terminal and start the ROS node:
```bash
cd catkin_ws/
catkin_make
source devel/setup.bash
rosrun ros_remote_robot_server ros_server.py
```

Finally, connect your robot to the server (don't forget to check the IP address of the server with `ifconfig`) and start publishing on the node `node_name`.

### The robot

For the development we used a Raspberry with Python 2.7 but you can use whatever architecture you prefer as long as you connect to the proper ROS node.
Below are the commands to run a robot equipped with a Raspberry.

First set the environment variables: in the file `file`, add the line `line`.

Now prepare the ROS environment:
```bash
mkdir -p ./catkin_ws/src
cd catkin_ws/
catkin_make
source devel/setup.bash
```

Put the content of `rasp/` in  `catkin_ws/src/` then make the file executable with:
```bash
chmod +x src/rasp/scripts/ros_package_raspberry.py
```

Connect to the server and launch the node:
```bash
export ROS_MASTER_URI=http://<ip>:<port>
rosrun rasp ros_package_raspberry.py ########################################### Nom package sur la raspbderry ??
```

### Virtual robot

If you wish to test the project and you don't have a robot, we built a virtual robot that simulates a real one. You just have to make the file executable and run it
```bash
cd catkin_ws/
source devel/setup.bash
chmod +x src/ros_remote_robot_server/scripts/ros_server.py ######################### Filename ??
catkin_make
rosrun ros_remote_robot_server ros_server.py ######################### Filename ??
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
- Compatibility in real time of 2 programs using different version of Python.
- Integration with third-party libraries.
- Collaborative development using GitHub, Pull Requests and Trello.

## Test commands

TODO for the other tests

To launch tests to test the socket between the ROS node and the Django server

`cd remote_robot_web/control_board`
`python -m unittest`

## Contributing

- Create a branch, e.g. `feature/new-feature` or `fix/very-nasty-bug`.
- Add commits.
- When ready, push to remote: `git push -u origin feature/new-feature`.
- Open a Pull Request: document the changes and provide any useful additional context.
- Ask someone to review your code.
- When ready: merge it.
