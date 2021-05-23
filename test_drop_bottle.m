clc, clear, close all
%% Connect to ROS Network
ros_ip = '192.168.0.2';
rosshutdown;

rosinit(ros_ip,11311);
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
%% Define position bottle
translation = [0.39 -0.17 -0.04]; 
orientation=[pi 0 -pi/2];

MTH_goal = trvec2tform(translation)*eul2tform(orientation,'XYZ');

joint_state_actual = receive(joint_state_sub,2);
joint_position =  joint_state_actual.Position(2:8);
moving_time = 2;

[q,qd,qdd,trajTimes] = computeTrajectory( joint_position, MTH_goal, robot, 'gripper', moving_time);                      
trajGoalMsg = packageJointTrajectory(trajGoalMsg,q,qd,qdd,trajTimes);

waitForServer(trajAct);
sendGoalAndWait(trajAct,trajGoalMsg);
SLActivateGripper("close");

drop_item("green",robot);