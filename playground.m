clc, clear, close all
%% Connect to ROS Network
%ipaddress = '192.168.182.128'; % VM
ipaddress = fileread("ROS_ip.txt");
rosshutdown;

rosinit(ipaddress,11311);
%% Testing ROS commands 
% rosaction("list")
% rostopic("info", "tf")
% rostopic("info", "clock") 
% svc_list =rosservice("list");
% rosservice("info","/gazebo/set_model_configuration")
% rosservice info '/gazebo/set_model_configuration'
%% Load robot model
load('exampleHelperKINOVAGen3GripperROSGazebo.mat');
% loadrobot("kinovaGen3");
% axes = show(robot);
%% Initialize 
physicsClient = rossvcclient('/gazebo/pause_physics');
call(physicsClient,'Timeout',3);

RoboCupManipulation_setInitialConfig; % DO NOT MODIFY
% Initial pose
% ["joint_1" "joint_2" "joint_3" "joint_4" "joint_5" "joint_6" "joint_7"]
% [ -1.9494   -0.0346   -1.1962   -1.0550    0.0367   -2.0500    1.5847]

%% Unpause gazebo
physicsClient = rossvcclient('gazebo/unpause_physics');
call(physicsClient,'Timeout',3);

%% Configure ROS subscriber 
jointSub = rossubscriber('/my_gen3/joint_states');
ros_action = '/my_gen3/gen3_joint_trajectory_controller/follow_joint_trajectory';
[trajAct,trajGoalMsg] = rosactionclient(ros_action);
% [client,goalMsg] = rosactionclient( actionName)
showdetails(trajGoal)
%% Define position 
base_pos=[-0.13, -0.1, 0.6];
bottle_pos=[0.26 -0.27 0.54];% Botella facil


translation=bottle_pos-base_pos; 

orientation=[pi 0 pi/2];
H_transform=trvec2tform(translation)*eul2tform(orientation,'XYZ')  

jointMsg = receive(jointSub,2);
jointPosition =  jointMsg.Position(2:8);

jointMsg.Name;

[q,qd,qdd,trajTimes] = computeTrajectory( jointPosition, H_transform, robot, 'gripper', 4);                      
%[q,qd,qdd,trajTimes] = computeTrajectory(...
%                         currentRobotJConfig, taskFinal, robot, endEffector, trajDuration)


N=length(trajTimes);

dq=[0;0;0;0;-2;0;0];              % movimiento relativo relativa
% qdd=0*qdd;
% qd=dq*ones(1,N);

q2=interp1([jointPosition,jointPosition+dq]',linspace(1,2,N),'linear');
q2=wrapToPi(q2)';

% Send position message
trajGoal = packageJointTrajectory(trajGoal,q,qd,qdd,trajTimes);

waitForServer(trajAct);
sendGoal(trajAct,trajGoal)
%waitForServer(trajAct);
%SLActivateGripper("close")

%drop_item("green")

%% Plot trajectory
close all
h=plot(trajTimes,(q2-jointPosition));
hold on
for k=1:7
  yline(jointPosition(k),"--","Color",h(k).Color)
end
legend(jointMsg.Name(2:8))
title("Joint Position  [rad]")
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
