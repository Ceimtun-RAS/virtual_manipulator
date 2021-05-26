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
joint_sub = rossubscriber('/my_gen3/joint_states');
ros_action = '/my_gen3/gen3_joint_trajectory_controller/follow_joint_trajectory';
[traj_client,traj_goal] = rosactionclient(ros_action);      % [client,goalMsg] = rosactionclient( actionName)
%showdetails(trajGoal)

rgbImgSub = rossubscriber('/camera/color/image_raw');     % camera sensor
rgbDptSub = rossubscriber('/camera/depth/image_raw');     % depth sensor
%%
close all

trajTimes=1;

q_home=[-1.9494;-0.0347;-1.1961;-1.0551;0.0367; -2.0500; 1.5847];
%q_home=[0;0.5;0;0.9;0;1.6;0];
qd=zeros(7,1);
qdd=zeros(7,1);

traj_goal = packageJointTrajectory(traj_goal,q_home,qd,qdd,trajTimes);
waitForServer(traj_client);
sendGoalAndWait(traj_client, traj_goal);
curImage = receive(rgbImgSub);
rgbImg = readImage(rgbImgSub.LatestMessage);
imshow(rgbImg)
%imwrite(rgbImg,'data/img_home.png')

%% sweep 
disp("sweep")
q_vel=[0.0;0;0;0;0;0;0];     % joint velocities  
qd=zeros(7,1);
qdd=zeros(7,1);
q=pi/180*[-90;-20;0;-65;0; 0; 0]; % sweep starting postion
traj_goal = packageJointTrajectory(traj_goal,q,qd,qdd,trajTimes);
waitForServer(traj_client);
sendGoalAndWait(traj_client, traj_goal);
%%
dt=1;

for k=1:5

joint_pos_reciver = receive(joint_sub,2); %Get current position in maximum 2 seconds
q=joint_pos_reciver.Position(2:8);

q=q+q_vel*dt;

qd=q_vel;
qdd=zeros(7,1);

traj_goal = packageJointTrajectory(traj_goal,q,qd,qdd,trajTimes);
waitForServer(traj_client);
sendGoalAndWait(traj_client, traj_goal);
curImage = receive(rgbImgSub);
rgbImg = readImage(rgbImgSub.LatestMessage);
imshow(rgbImg)
pause(1);
end
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
