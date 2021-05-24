clc, clear, close all
%% Connect to ROS Network
ros_ip = '192.168.0.2';
rosshutdown;

rosinit(ros_ip,11311);
%% Load robot model and initialize
load('exampleHelperKINOVAGen3GripperROSGazebo.mat');
RoboCupManipulation_setInitialConfig;
physics_client = rossvcclient('gazebo/unpause_physics');
call(physics_client,'Timeout',3);
%% Configure ROS subscriber 
joint_sub = rossubscriber('/my_gen3/joint_states');
ros_action = '/my_gen3/gen3_joint_trajectory_controller/follow_joint_trajectory';
[traj_client,traj_goal] = rosactionclient(ros_action);
%% Define movement
% Main positions
base_pos=[-0.13, -0.1, 0.6];
bottle_pos=[0.26 -0.27 0.56];% Botella facil
bin_pos = [-0.3 -0.4 0.1];

%MTH from base to bottle
translation=bottle_pos-base_pos; 
orientation=[pi 0 -pi/2];
MTH_base_bott = trvec2tform(translation)*eul2tform(orientation,'XYZ');  

% Get current robot position 
current_joint_pos = receive(joint_sub,2); %Get current position in maximum 2 seconds
joint_position =  current_joint_pos.Position(2:8);

[q,qd,qdd,trajTimes] = computeTrajectory(joint_position, MTH_base_bott, robot, 'gripper', 4);                      
%[q,qd,qdd,trajTimes] = computeTrajectory(currentRobotJConfig, taskFinal, robot, endEffector, trajDuration)

% Move robot
traj_goal = packageJointTrajectory(traj_goal,q,qd,qdd,trajTimes);
q=q';

waitForServer(traj_client);
sendGoal(traj_client, traj_goal);
%% Plot trajectory
plot_traj(q', robot, 'gripper')
