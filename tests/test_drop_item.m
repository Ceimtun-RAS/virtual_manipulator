clc, clear, close all
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

%% Define position bottle
run trajectory.m
clear data_log
data_log.translation=[];
data_log.orientation=[];
data_log.bin=[];
data_log.img=[];
data_log.depth=[];

base_pos=[-0.13, -0.1, 0.6];
offset=[0,0,0.04];
N=size(v_translation,1);
N=3;
for k=4:6
  disp("Step "+k+" of "+N)
  pause(3)
  translation = v_translation(k,:)-base_pos+offset; 
  orientation=v_orientation(k,:);
  bin=v_bin(k);
  drop_item(translation,orientation,bin,robot);
  
%   MTH_goal = trvec2tform(translation)*eul2tform(orientation,'XYZ');
% 
%   joint_state_actual = receive(joint_state_sub);
%   joint_position =  joint_state_actual.Position(2:8);
%   moving_time = 1;
% 
%   [q,qd,qdd,trajTimes] = computeTrajectory( joint_position, MTH_goal, robot, 'gripper', moving_time);                      
%   trajGoalMsg = packageJointTrajectory(trajGoalMsg,q,qd,qdd,trajTimes);
% 
%   waitForServer(trajAct);
%   sendGoalAndWait(trajAct,trajGoalMsg);
%   if(mod(k,2)==0)
%     SLActivateGripper("close");
% 
%     % Retrieve sensor data_log
%     curImage = receive(rgbImgSub);
%     rgbImg = readImage(rgbImgSub.LatestMessage);
%     curDepth = receive(rgbDptSub);
%     depthImg = readImage(rgbDptSub.LatestMessage); 
%     img_path="../data/image"+k+".png"
%     imwrite(rgbImg,img_path);
% 
%     data_log(k).translation=translation;
%     data_log(k).orientation=orientation;
%     data_log(k).bin=bin;
%     data_log(k).img=rgbImg;
%     data_log(k).depth=depthImg;

    %Send to bin
%     drop_item(bin,robot);
%   end
end 

%save ../data/sensor_data.mat data_log
%% 
% close all
% load ../data/sensor_data.mat
% imshow(data_log(1).img)
% figure()
% 
% imshow(data_log(1).depth)
%%
% close all
% %images=zeros(size(data_log(1).img));
% 
% images=[]
% depth=[];
% for k=1:size(data_log,2)
%   images(:,:,:,k)=data_log(k).img;
%   depth(:,:,k)=data_log(k).depth;
% end
% 
% 
% implay(images,1)
% %implay(depth,1)

% 
% trajTimes=1;
% 
% q_home=[-1.9494;-0.0347;-1.1961;-1.0551;0.0367; -2.0500; 1.5847];
% %q_home=[0;0.5;0;0.9;0;1.6;0];
% qd=zeros(7,1);
% qdd=zeros(7,1);
% 
% trajGoalMsg = packageJointTrajectory(trajGoalMsg,q_home,qd,qdd,trajTimes);
% waitForServer(trajAct);
% sendGoalAndWait(trajAct, trajGoalMsg);
% curImage = receive(rgbImgSub);
% rgbImg = readImage(rgbImgSub.LatestMessage);
% imshow(rgbImg)
