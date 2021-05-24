clc, clear, close all
%% Define movement
load('exampleHelperKINOVAGen3GripperROSGazebo.mat');
% Main positions
base_pos=[-0.13, -0.1, 0.6];
bottle_pos=[0.26 -0.27 0.56];% Botella facil
bin_pos = [-0.3 -0.4 0.1];

%MTH from base to bottle
translation=bin_pos; 
orientation=[pi 0 -pi/2];
MTH_base_bott = trvec2tform(translation)*eul2tform(orientation,'XYZ');  

% Get current robot position 
%joint_position=[ -1.9494   -0.0346   -1.1962   -1.0550    0.0367   -2.0500    1.5847]'
joint_position=[-2.37997455034067	-0.951112155576302	-0.432600110536910	-1.94594577779313	0.902245097515458	-0.449930323956808	1.21967465467051]';

[q,qd,qdd,trajTimes] = compute_trajectory(joint_position, MTH_base_bott, robot, 'gripper', 2);                     
%% Plot trajectory
plot_traj(q, robot, 'gripper')
