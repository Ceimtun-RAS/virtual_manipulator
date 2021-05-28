clc, clear, close all
%% Connect to ROS Network
ros_ip = fileread("ROS_ip.txt");
rosshutdown;

rosinit(ros_ip,11311);
%% Load robot model
load('exampleHelperKINOVAGen3GripperROSGazebo.mat');
%% Configure ROS subscriber 
RoboCupManipulation_setInitialConfig;

physicsClient = rossvcclient('gazebo/unpause_physics');
call(physicsClient,'Timeout',3);

joint_state_sub = rossubscriber('/my_gen3/joint_states');
ros_action = '/my_gen3/gen3_joint_trajectory_controller/follow_joint_trajectory';
[traj_client,traj_goal] = rosactionclient(ros_action);

rgbImgSub = rossubscriber('/camera/color/image_raw');     % camera sensor
rgbDptSub = rossubscriber('/camera/depth/image_raw');     % depth sensor

%% Initialize 
% RoboCupManipulation_setInitialConfig;
% 
% q_home=[-1.9494;-0.0347;-1.1961;-1.0551;0.0367; -2.0500; 1.5847];
% qd=zeros(7,1);
% qdd=zeros(7,1);
% 
% traj_goal = packageJointTrajectory(traj_goal,q_home,qd,qdd,1);
% waitForServer(traj_client,2);
% sendGoalAndWait(traj_client,traj_goal);
%   

%% Define position bottle

endEffector='gripper';
is_relative=0;
is_targeted=0;
for var=1:10
  disp("Step")
  pause(1)
  
  curImage = receive(rgbImgSub);
  rgbImg = readImage(rgbImgSub.LatestMessage);
  % movement direction determine by the image
  x=-0.02;
  y=0.04;
  z=-0.02;
  weights = [1 1 0.1 1 1 0.1];

  
  moving_time = 2;

  
  translation=[x,y,z]*moving_time;
  orientation=[0,0,0];
  
%   translation = [0.3 -0.4+0.1*var 0.1];
%   orientation = [pi 0 -pi/2];
    
%   if is_targeted 
%     curDepth = receive(rgbDptSub);
%     depthImg = readImage(rgbDptSub.LatestMessage); 
%   
%     z=-0.1;
%     yaw=0;
%     translation=[0,0,z];
%     orientation=[0,0,yaw];
%   end
%   
  H_transform = trvec2tform(translation)*eul2tform(orientation,'XYZ');
 
  % get arm position 
  %waitForServer(joint_state_sub,2);
  
  joint_state_actual = receive(joint_state_sub,2);
  joint_position =  joint_state_actual.Position(2:8);
  joint_init =wrapToPi(joint_position');
  T0 = getTransform(robot, joint_init, endEffector);          % starting homogeneus transform
  TF = H_transform*T0;

  
  
  
  [q,qd,qdd,trajTimes] = compute_trajectory_mod(weights,...
                          joint_init, T0,TF,...
                          robot, 'gripper', moving_time); 
  
                           
  traj_goal = packageJointTrajectory(traj_goal,q,qd,qdd,trajTimes);
  waitForServer(traj_client,2);
  sendGoalAndWait(traj_client,traj_goal);
  
  %SLActivateGripper("close");

    % Retrieve sensor data_log
%    img_path="../data/image"+k+".png"
%     imwrite(rgbImg,img_path);
% 
%     data_log(k).translation=translation;
%     data_log(k).orientation=orientation;
%     data_log(k).bin=bin;
%     data_log(k).img=rgbImg;
%     data_log(k).depth=depthImg;

    %Send to bin
    bin="green";
    %drop_item(bin,robot);

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

