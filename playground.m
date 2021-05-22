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
svc_list =rosservice("list");
rosservice("info","/gazebo/set_model_configuration")
rosservice info '/gazebo/set_model_configuration'
clc

%% load robot model

load('exampleHelperKINOVAGen3GripperROSGazebo.mat');
%loadrobot("kinovaGen3");
%axes = show(robot);

%% Initialize 
physicsClient = rossvcclient('/gazebo/pause_physics');
physicsResp = call(physicsClient,'Timeout',3);

RoboCupManipulation_setInitialConfig; % DO NOT MODIFY
%initial pos [ -1.9494   -0.0346   -1.1962   -1.0550    0.0367   -2.0500    1.5847]; 
% "joint_1", "joint_2", "joint_3","joint_4", "joint_5", "joint_6", "joint_7"

%% unpause gazebo
physicsClient = rossvcclient('gazebo/unpause_physics');
physicsResp = call(physicsClient,'Timeout',3);


%% Configure ROS subscriber 
jointSub = rossubscriber('/my_gen3/joint_states');
ros_action='/my_gen3/gen3_joint_trajectory_controller/follow_joint_trajectory';
[trajAct,trajGoal] = rosactionclient( ros_action);
% [client,goalMsg] = rosactionclient( actionName)
showdetails(trajGoal)

%% Define position 

base_pos=[-0.13, -0.1, 0.6];
bottle_pos=[0.26 -0.27 0.54];
bluebin_pos=[-0.3 -0.4 0.4];
greenbin_pos=[-0.3 0.4 0.4];
%translation=[-0.3 0.4 0.4];
translation=bottle_pos-base_pos; % Botella facil
translation=bluebin_pos;

orientation=[pi 0 pi/2];
H_transform=trvec2tform(translation)*eul2tform(orientation,'XYZ')  

jointMsg = receive(jointSub,2);
jointPosition =  jointMsg.Position(2:8);

jointMsg.Name;

[q,qd,qdd,trajTimes] = RoboCupManipulation_computeTrajectory( ...
                        jointPosition, H_transform, robot, 'gripper', 4);                      
%[q,qd,qdd,trajTimes] = RoboCupChallenge_computeTrajectory(...
%                         currentRobotJConfig, taskFinal, robot, endEffector, trajDuration)

N=length(trajTimes);
trajGoal = packageJointTrajectory(trajGoal,q,qd,qdd,trajTimes);

waitForServer(trajAct);
sendGoal(trajAct,trajGoal)
pause(4)
SLActivateGripper("close")
pause(4)
SLActivateGripper("open")

%% Plot trajectory
close all
h=plot(trajTimes,q');
hold on
for k=1:7
  yline(jointPosition(k),"--","Color",h(k).Color)
end
legend(jointMsg.Name(2:8))
title("Joint Postion  [rad}")
hold off
% 
% plot(qd')
% title("Joint Velocity  [rad/s}")
% 
% subplot(2,2,4)
% plot(qdd')
% title("Joint acceleration  [rad/s^2}")

%% Send position message 
% trajGoal = packageJointTrajectory(trajGoal,q,qd,qdd,trajTimes);
% 
% waitForServer(trajAct);
% sendGoal(trajAct,trajGoal)

%% Recieve Camera 

% rgbImgSub = rossubscriber('/camera/color/image_raw');
% curImage = receive(rgbImgSub);
% rgbImg = readImage(rgbImgSub.LatestMessage); 
% imshow(rgbImg)

