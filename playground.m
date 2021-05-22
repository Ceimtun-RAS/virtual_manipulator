%% Connect to ROS Network
clc, clear 
ipaddress = '192.168.182.128';

%ipaddress = '10.0.3.25';

rosshutdown
rosinit(ipaddress,11311);

%% testing ROS commands 

rosaction("list")
rostopic("info", "tf")
rostopic("info", "clock") 
svclist =rosservice("list");
rosservice("info","/gazebo/set_model_configuration")
rosservice info '/gazebo/set_model_configuration'
clc

%% load robot model

load('exampleHelperKINOVAGen3GripperROSGazebo.mat');
%axes = show(robot);

%% Initialize 

RoboCupManipulation_setInitialConfig; % DO NOT MODIFY
%% unpause gazebo
physicsClient = rossvcclient('gazebo/unpause_physics');
physicsResp = call(physicsClient,'Timeout',3);
%% Configure ROS subscriber 
jointSub = rossubscriber('/my_gen3/joint_states');
ros_action='/my_gen3/gen3_joint_trajectory_controller/follow_joint_trajectory';
[trajAct,trajGoal] = rosactionclient( ros_action);
showdetails(trajGoal)

%% Define position 

translation=[0.23 0 0.06];
angles=[0 pi pi/2];
H_transform=trvec2tform(translation)*eul2tform(angles,'XYZ')  

jointMsg = receive(jointSub,2);
jointPosition =  jointMsg.Position(2:8);

jointMsg.Name;

[q,qd,qdd,trajTimes] = RoboCupManipulation_computeTrajectory( ...
                        jointPosition, H_transform, robot, 'gripper', 4);
N=length(trajTimes);
%% Plot trajectory
close all
h=plot(trajTimes,q');
hold on
for k=1:7
  yline(jointPosition(k),"--","Color",h(k).Color)
end
legend(jointMsg.Name(2:8))
title("Joint Postion  [rad}")

% 
% plot(qd')
% title("Joint Velocity  [rad/s}")
% 
% subplot(2,2,4)
% plot(qdd')
% title("Joint acceleration  [rad/s^2}")

%% Send position message 
trajGoal = RoboCupManipulation_packageJointTrajectory(trajGoal,q,qd,qdd,trajTimes);
trajGoal = packageJointTrajectory(trajGoal,q,qd,qdd,trajTimes);

waitForServer(trajAct);
sendGoal(trajAct,trajGoal)

%% Recieve Camera 

rgbImgSub = rossubscriber('/camera/color/image_raw');
curImage = receive(rgbImgSub);
rgbImg = readImage(rgbImgSub.LatestMessage); 
imshow(rgbImg)

