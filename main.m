clc, clear, close all

addpath('functions','vision_','data')  

%% Connect to ROS Network
ros_ip = fileread("ROS_ip.txt");
rosshutdown;

rosinit("192.168.182.129",11311);
%% Load robot model
load('exampleHelperKINOVAGen3GripperROSGazebo.mat');
%% Initialize 
RoboCupManipulation_setInitialConfig;
physicsClient = rossvcclient('gazebo/unpause_physics');
call(physicsClient,'Timeout',3);
%% Configure ROS subscriber 
joint_state_sub = rossubscriber('/my_gen3/joint_states');
ros_action = '/my_gen3/gen3_joint_trajectory_controller/follow_joint_trajectory';
[trajAct,trajGoalMsg] = rosactionclient(ros_action);

rgbImgSub = rossubscriber('/camera/color/image_raw');     % camera sensor
rgbDptSub = rossubscriber('/camera/depth/image_raw');     % depth sensor

%% Test image processing

curImage = receive(rgbImgSub);
img = readImage(rgbImgSub.LatestMessage);

curDepth = receive(rgbDptSub);
I = readImage(rgbDptSub.LatestMessage); 
I=uint8(I*255);

[axes, ptsCloud] = getAxis(img, I);
mid_pts = midPointAxis(axes);
hold on; 

% Plot axis 
% scatter3(mid_pts(:, 1), mid_pts(:, 2),mid_pts(:, 3), 500, '.'); 

% collision boxes 

[N, val] = size(axes);
% collisionboxes = zeros(N, 1);
clear collisionboxes;
for i = 1:N
    collisionboxes(i) = CreateCollisionBoxFromAxis(axes(i, :))
    %hold on; 
    %show(collisionboxes); 
end

distance=vecnorm(mid_pts(:,1:2),2,2);

target=mid_pts(1,:)
K=[0.2,0.1,0]/480;        % Proportional constant [x,y,z]; 

translation=K.*target;    % Relative position
orientation=[pi, 0, 0];
bin="green";

% 
% jointMsg = receive(joint_state_sub,2);
% jointPosition =  jointMsg.Position(2:8);
% jointInit=wrapToPi( jointPosition');
% 
% T0 = getTransform(robot, jointInit, 'gripper');
% robot_position=tform2trvec(T0);
% translation=translation + robot_position;
%drop_item(translation,orientation,bin,robot);


%% Define position bottle
run trajectory.m

base_pos=[-0.13, -0.1, 0.6];
offset=[0,0,0.025];
N=size(v_translation,1);
%N=3;

for k=1:N
  disp("Step "+k+" of "+N)
  pause(3)
  translation = v_translation(k,:)-base_pos+offset; 
  orientation=v_orientation(k,:);
  bin=v_bin(k);
  drop_item(translation,orientation,bin,robot);
  
end 

