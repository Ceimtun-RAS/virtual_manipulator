clc, clear, close all
%% Define movement
load('exampleHelperKINOVAGen3GripperROSGazebo.mat');

%MTH from base to bottle
translation = [-0.3 -0.4 0.1]; 
orientation = [pi 0 -pi/2];
MTH_base_bott = trvec2tform(translation)*eul2tform(orientation,'XYZ');  

% Get current robot position 
joint_position=[ -1.9494   -0.0346   -1.1962   -1.0550    0.0367   -2.0500    1.5847]'

[q,qd,qdd,trajTimes] = compute_trajectory(joint_position, MTH_base_bott, robot, 'gripper', 2);                     
%% Plot trajectory
plot_traj(q, robot, 'gripper')