###### RCP - Robot Configuration Parser

https://github.com/aaai-disim-univaq
```
Project:
- physical robotics

Authors:
- Valentino Di Giosaffatte
- Riccardo Armando Di Prinzio

Course:
- Intelligent Systems And Robotics Laboratory

Supervised by:
- Prof. Giovanni De Gasperis

Master Degree in Computer Science
Curriculum: Network and Data Science

University of L'Aquila
Department of Information Engineering, Computer Science and Mathematics
```
---
##### info
```
rcp is a parser and code generator that works with abstract robot descriptions written in yaml format.

rcp is capable of generating 2 separate libraries which provide an abstraction layer between a ros2 
environment and a non-ros environment

this layer guarantees to 2 separate programs (one on the non-ros side and the other on the ros2 side) 
a bidirectional communication in a distributed environment and in a bidirectional way

the layer provides an abstraction for the payloads (automatic conversion of ros2 messages with custom 
interface in JSON format), this allows us to control the actuators and sensors of a robot that does not 
have a ros2 implementation through the use of a packaged ros2 program
```
---
##### ros2 implemented custom message interfaces (from ros2 message broker, middleware)
```
ACTUATORS
    geometry_msgs / msg / Twist
    
SENSORS
    ...
```
---
##### fast usage
```
example:
- parses the robot configuration file 'config/freenove.yml'
- generates the code of both libraries
- builds and runs a host example that uses the non-ros library
- instantiate a distributed docker infrastructure (virtual machines: redis, ros2 dashing)
- builds and runs a ros2 package inside the ros2 dashing virtual machine using the ros2 library
```
https://github.com/mdegree-nedas/rcp/tree/master/docs/getting_started

---
##### code generation
---
###### 1 -- define the robot configuration yml file
###### (yml file structure)
```yaml
<robot-name>:
  message_broker_ip: <ip>
  message_broker_port: <port>
  sensors:
    <sensor-name>:
      id: <id>
      type: <type>
      address: <address>
      topic: <topic>
      time: <time>
    <sensors-list ...>
  actuators:
    <actuator-name>:
      id: <id>
      address: <address>
      topic: <topic>
      commands:
        - <command-1>
        - <command-2>
        - <command-list ...>
    <actuators-list ...>
  commands:
    <command-1>:
      data: <data>
      time: <time>
    <command-2>:
      data: <data>
      time: <time>
    <command-list ...>
```
---
###### 2 -- generate the code
###### (rcp call synopsis, single config file)
```console
user@hostname:~$ python rcp.py config-file.yml
```
###### (rcp call synopsis, multiple config files)
```console
user@hostname:~$ python rcp.py config-file-1.yml config-file-2.yml ...
```
---
###### 3 -- generated code, filesystem tree
```
out
├── <robot-name-1>
│   ├── noros
│   │   ├── __init__.py
│   │   ├── broker.py
│   │   ├── core.py
│   │   └── template.py
│   └── ros
│       ├── __init__.py
│       ├── broker.py
│       ├── core.py
│       └── interface.py
├── <robot-name-2>
│   ├── noros
│   │   ├── __init__.py
│   │   ├── broker.py
│   │   ├── core.py
│   │   └── template.py
│   └── ros
│       ├── __init__.py
│       ├── broker.py
│       ├── core.py
│       └── interface.py
├── ...
...
```
---
##### usage
---
###### ROS2 package (example)
###### project tree
```
<ros2-package>
├── __init__.py
├── main.py
└── ros-generated-lib
    ├── __init__.py
    ├── core.py
    ├── broker.py
    └── interface.py
```
###### Host (example)
###### project tree
```
<host-venv>
├── __init__.py
├── main.py
└── host-generated-lib
    ├── __init__.py
    ├── core.py
    ├── broker.py
    └── template.py
```
