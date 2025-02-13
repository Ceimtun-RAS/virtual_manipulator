
# Robocup virtual manipulator challenge 
![](images/hero.png)
## Resources 
*	[Virtual manipulator challenge main page](https://2021.robocup.org/robot-manipulation) 
* [Video introduction](https://youtu.be/h-IVj2tajQQ?t=1561)
* [Robocup@work team qualifications](https://atwork.robocup.org/2021/03/12/robocup-2021-worldwide-call-for-participation/) 
* [Robocup@home team qualifications](https://athome.robocup.org/2021-cfp-all/)
* [Github getting started](https://github.com/mathworks-robotics/templates-robocup-robot-manipulation-challenge)
*	[Design and Control of Robot Manipulators ](https://www.facebook.com/notes/matlab-and-simulink-robotics-arena/design-and-control-of-robot-manipulators-technical-resources/3351011848336733/) 
* [Get started with gazebo](https://www.mathworks.com/help/ros/ug/get-started-with-gazebo-and-a-simulated-turtlebot.html)
* [Blog](https://jsduenass.github.io/posts/matlab/)

### Other resources 
* [Pick-and-Place Workflow in Gazebo Using Point-Cloud Processing and RRT Path Planning](https://www.mathworks.com/help/robotics/ug/pick-and-place-gazebo-with-point-clouds-and-rrt.html)

## Objective
Crate an algorithm for the robotic arm so it can sort properly the cans and bottles. 
* Cans must go into the green bin
* Bottles must go into the blue bin 

## Config of this project
* A submodule is used the repository [vision_](https://github.com/mora200217/vision_) if you want to clone this repository use the command ```git clone --recurse-submodules https://github.com/jsduenass/virtual_manipulator```
* add the folders ```data``` and ```functions``` to the matlab path.
* if you need to change the ROS ip address run the following command so git ignores the changes```git update-index --skip-worktree data/ROS_ip.txt```. If you need to update the submodule ```git submodule update --remote --merge```


## About the virtual machine

The work is based on the vmware virtual machine provided by the github getting started. It contains: 
* ROS 2 Dashing desktop installation
* ROS Melodic desktop installation
* Gazebo robot simulator 9.0.0
* Example Gazebo worlds for a simulated TurtleBot® 3

Checking the version of the software packages
```
lsb_release -a
gazebo -v
```

* Ubuntu version: 18.04.5
* Gazebo version: 9.16.0

The project uses ros_kortex  robotic arm for the virtual manipulator  


There are different platforms where to run vmware workstation player and virtual box. The tutorial recommends using the vmware route and issues where found while using virtual box

### vmware 
While running the virtual machine some errors were encounter  
```
Error while powering on: VMware Player does not support nested virtualization on this host.
```
Which were fix by disabling the ```virtual CPU performance counters``` and ```Virtual Intel VT-x/EPT or AMD-V/RVI``` settings 
and ```disable side channel mitigation```

### Virtual box (Not recommended)
* Running and testing the vm (virtual machine) works
* Take a snapshot of the vm in order to have a backup to go back to if something goes wrong.  
* Using virtual box Guest additions to get full-screen 
* Activate share clipboard to copy paste to work between both in host and guest
* Configure Network between guest and host. Here there are two options:
  * Enable network adapter with bridge configuration. issue: scripts file where the ros_master URI is configured fail because many network interfaces are recognized so the ros_master URI needs to be manually defined.  
  ![Network configuration](images/network-config.png)
  * Enable network adapter with __NAT__ configuration and allow __port forwarding__ in advance settings. The port 11311 needs to be forward in order for the host computer to connect to ROS network.  NAT (Network Address Translation) creates a private network for the guest vm So the IP address used from inside the guest ```192.168.56.1``` is different tha the used by the host computer ```192.168.56.1```. issue: ROS services does not working properly with MATLAB. Note: Check the virtual box network tool to identify network adaptor of the host computer ```192.168.56.1```


Once a virtual machine is setup and running some minor configuration are made in order to make it easier to use

* Add [Spanish keyboard](https://askubuntu.com/questions/1014585/how-to-add-a-latin-american-keyboard-in-17-10): changing the default keyboard layout from English to Spanish  simplifies the use of special characters

  ```
  sudo locale-gen es_AR.UTF-8
  ```

* Add the setup file from ROS and catkin environment to bash, in order to add the ROS environment variable and be able to run ROS command from the shell. 
  ```
  echo 'source /opt/ros/melodic/setup.bash' >>  ~/.bashrc
  echo 'source ~/catkin_ws/devel/setup.bash' >>  ~/.bashrc
  ```
* try running ```rostopic list``` to test it is working 


## Environment exploration
Files related to the project are located at the Desktop inside the Robocup Challenge folder. Exploring these files ( ```Example World 1.desktop```) from the command line give an insight in how they work
```
cd "~/Desktop/RoboCup Challenge"
less "Example World 1.desktop"
```

Content of  ```Example World 1.desktop```: 
```
[Desktop Entry]
Version=1.0
Comment=Launch example world for RoboCup challenge
Exec=/home/user/start-robocup-example-world-1.sh
Terminal=false
Type=Application
Categories=Utility;Application;
Icon=/usr/share/icons/Tango/32x32/mimetypes/gnome-mime-application-x-executable.png
Name[en_US]=Example World 1
```

This file launches a  shell script ```~/start-robocup-example-world-1.sh```

```
less ~/start-robocup-example-world-1.sh
```

Content of ```~/start-robocup-example-world-1.sh```:
```
#!/bin/sh
# Shortcut shell script to start pickandplace recycling world
export SVGA_VGPU10=0
export ROS_IP=$(hostname -I | tr -d [:blank:])
export ROS_MASTER_URI=http://$ROS_IP:11311
export GAZEBO_PLUGIN_PATH=/home/user/src/GazeboPlugin/export:$GAZEBO_PLUGIN_PATH
# Launch Gazebo world with kortex
gnome-terminal --title="Gazebo Recycling World - Depth Sensing" -- /bin/bash -c 'source /opt/ros/melodic/setup.bash; source ~/catkin_ws/devel/setup.bash; roslaunch kortex_gazebo_depth pickplace.launch world:=RoboCup_1.world '
```

This file open a gazebo world from the ROS project found at  ```~/catkin_ws/```


search for the robot model (urdf) 
```
cd ~/catkin_ws
ls -R | grep urdf
```

```./src/ros_kortex/kortex_description/arms/gen3/urdf```

### Connect to MATLAB 
* Select on of the available example files located on the RoboCup Challenge folder, it would run a gazebo world and start ROS.
* Open MATLAb and run the following script to connect to 

```{matlab}
%% Connect to ROs Network
clc
ipaddress = '192.168.182.128';

rosshutdown
rosinit(ipaddress,11311);

% Initializing global node /matlab_global_node_47691 with NodeURI http://192.168.56.1:59106/
```
The MATLAB command ```rosshutdown``` ends the connection
 
ROS action client 

### Basic ROS commands 
```
%% testing ROS commands 
rosaction("list")
rostopic("info", "tf")
rostopic("info", "clock")
svclist =rosservice("list");
rosservice("info","/gazebo/set_model_configuration")
rosservice info '/gazebo/set_model_configuration'
```
## Important information 
variables of interest:
* rostopic: '/my_gen3/joint_states'
  * message type: 'sensor_msgs/JointState'


```rqt_graph```: Show a graphical representation of the ROS network


matlab homogeneous transformation matrix
euler angles 
intrinsic rotation

## ROS concepts

* ROS Node
* ROS Publisher: Broadcast information
* ROS Subscriber: Request information
* ROS message: packet of information

axis color 
* R -> x, roll
* G -> y, pitch 
* B -> z, yaw

## Main items position

__Base_link__ : [-0.13, -0.1, 0.6]

![Objects in grey zone](images/fixed_objects.png)


1. Blue bottle <br/>
[-0.1592 -0.6765 0.6338] [0.00 -0.00 -0.0001]

2. Green can <br/>
    [-0.1065 0.363 0.6743] [0.0009 -0.0135 1.3401]

3. Blue bottle <br/>
    [0.0735 0.2639 0.6338] [0.00 -0.00 0.003]

4. Yellow bottle <br/>
    [-0.1430 0.5261 0.5557] [1.5592 1.4658 1.5682]

5. Red can vertical <br/>
    [0.2140 0.4408 0.5854] [3.1301 -0.0084 -3.0172] 

6. Red bottle <br/>
    [0.2214 0.5998 0.5559] [1.5708 -0.4342 1.5320]


[ROS service MATLAB](https://www.mathworks.com/help/ros/ref/rosservice.html)




