function drop_item(translation,orientation, bin_name, robot)
    % ROS config subscribers
    jointSub = rossubscriber('/my_gen3/joint_states');
    ros_action='/my_gen3/gen3_joint_trajectory_controller/follow_joint_trajectory';
    [trajAct,trajGoal] = rosactionclient(ros_action);
    
    safe_h=0.2;    
    upper_trans= translation + [0,0,safe_h];
    %upper_trans(3)=safe_h;
    
    % Ubicate over the object
           
    H_transform = trvec2tform(upper_trans)*eul2tform(orientation,'XYZ');
 
    jointMsg = receive(jointSub,2);
    jointPosition =  jointMsg.Position(2:8);
    
    move_time = 2;
    [q,qd,qdd,trajTimes] = computeTrajectory( ...
                            jointPosition, H_transform, robot, 'gripper', move_time);                      

    trajGoal = packageJointTrajectory(trajGoal,q,qd,qdd,trajTimes);
    waitForServer(trajAct);
    sendGoalAndWait(trajAct,trajGoal);
    
    % desend
    H_transform = trvec2tform(translation)*eul2tform(orientation,'XYZ');
 
    jointMsg = receive(jointSub,2);
    jointPosition =  jointMsg.Position(2:8);
    
    move_time = 1;
    [q,qd,qdd,trajTimes] = computeTrajectory( ...
                            jointPosition, H_transform, robot, 'gripper', move_time);                      

    trajGoal = packageJointTrajectory(trajGoal,q,qd,qdd,trajTimes);
    waitForServer(trajAct);
    sendGoalAndWait(trajAct,trajGoal);
    
    SLActivateGripper("close");
    
    % asend 
    H_transform = trvec2tform(upper_trans)*eul2tform(orientation,'XYZ');
 
    jointMsg = receive(jointSub,2);
    jointPosition =  jointMsg.Position(2:8);
    
    move_time = 1;
    [q,qd,qdd,trajTimes] = computeTrajectory( ...
                            jointPosition, H_transform, robot, 'gripper', move_time);                      

    trajGoal = packageJointTrajectory(trajGoal,q,qd,qdd,trajTimes);
    waitForServer(trajAct);
    sendGoalAndWait(trajAct,trajGoal);
    
    % go to bin
    h=upper_trans(3);
    if bin_name == "blue"
        bin_pos = [-0.3 -0.45 h];
    elseif bin_name == "green"
        bin_pos = [-0.3 0.45 h];
    end
    
    H_transform = trvec2tform(bin_pos)*eul2tform(orientation,'XYZ');

    jointMsg = receive(jointSub,2);
    jointPosition =  jointMsg.Position(2:8);

    [q,qd,qdd,trajTimes] = computeTrajectory( ...
                            jointPosition, H_transform, robot, 'gripper', move_time);                      

    trajGoal = packageJointTrajectory(trajGoal,q,qd,qdd,trajTimes);
    waitForServer(trajAct);
    sendGoalAndWait(trajAct,trajGoal);
    
    SLActivateGripper("open");
end
