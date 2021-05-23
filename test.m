clear
clc
close all
%% Init ros node
%rosIP = "192.168.152.129";%WM
rosIP = "192.168.0.2" % Local
rosshutdown; % Shutdown previous MATLAB nodes 

rosinit(rosIP,11311); 
%% Load robot model
load('exampleHelperKINOVAGen3GripperROSGazebo.mat');
%% Initialize model and start simulation
% Initial configuration from RoboCup competition
RoboCupManipulation_setInitialConfig;

% Unpause physics
physicsClient = rossvcclient('gazebo/unpause_physics');
physicsResp = call(physicsClient,'Timeout',3);

%% Suscribers for control
jointSub = rossubscriber('/my_gen3/joint_states');
[trajAct,trajGoal] = rosactionclient( '/my_gen3/gen3_joint_trajectory_controller/follow_joint_trajectory') % [client,goalMsg] = rosactionclient(___)

%% Define gripper pose
gripperTranslation = [1 0 2]; % meters 
gripperRotation = [0 pi 0]; % radians Euler form
desiredGripperPose = trvec2tform(gripperTranslation) * eul2tform(gripperRotation);


jointMsg = receive(jointSub,2); % msg = receive(sub,timeout)
currentRobotJConfig =  jointMsg.Position(2:8);
[q,qd,qdd,trajTimes] = RoboCupManipulation_computeTrajectory(currentRobotJConfig, desiredGripperPose, robot, 'gripper', 4); % function [q,qd,qdd,trajTimes] = MWRoboCupChallenge_computeTrajectory(currentRobotJConfig, taskFinal, robot, endEffector, trajDuration)

trajGoal = RoboCupManipulation_packageJointTrajectory(trajGoal,q,qd,qdd,trajTimes);
waitForServer(trajAct);
sendGoal(trajAct,trajGoal)
%% Control Gripper Using ROS Action Client
[gripAct,gripGoal] = rosactionclient('/my_gen3/custom_gripper_controller/gripper_cmd');
gripperCommand = rosmessage('control_msgs/GripperCommand');
gripperCommand.Position = 0.02; % 0.04 fully closed, 0 fully open
gripperCommand.MaxEffort = 500;
gripGoal.Command = gripperCommand;
waitForServer(gripAct);
sendGoal(gripAct,gripGoal)
%% Cameras
% Receive Camera Image Using ROS Subscriber
figure
rgbImgSub = rossubscriber('/camera/color/image_raw');
curImage = receive(rgbImgSub);
rgbImg = readImage(rgbImgSub.LatestMessage); 
imshow(rgbImg)

% Receive Depth Image Using ROS Subscriber
figure
rgbDptSub = rossubscriber('/camera/depth/image_raw');
curDepth = receive(rgbDptSub);
depthImg = readImage(rgbDptSub.LatestMessage); 
imshow(depthImg)