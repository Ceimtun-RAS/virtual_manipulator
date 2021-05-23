function drop_item(bin_name, robot)
    jointSub = rossubscriber('/my_gen3/joint_states');
    ros_action='/my_gen3/gen3_joint_trajectory_controller/follow_joint_trajectory';
    
    move_time = 2;    
    [trajAct,trajGoal] = rosactionclient(ros_action);
    orientation = [pi 0 -pi/2];
    
    if bin_name == "blue"
        lift_pos = [0 -0.4 0.4];
        bin_pos = [-0.3 -0.4 0.1];
    elseif bin_name == "green"
        lift_pos = [0 0.4 0.4];
        bin_pos = [-0.3 0.4 0.1];
    end
    
    H_transform = trvec2tform(lift_pos)*eul2tform(orientation,'XYZ');

    jointMsg = receive(jointSub,2);
    jointPosition =  jointMsg.Position(2:8);

    [q,qd,qdd,trajTimes] = computeTrajectory( ...
                            jointPosition, H_transform, robot, 'gripper', move_time);                      

    trajGoal = packageJointTrajectory(trajGoal,q,qd,qdd,trajTimes);

    waitForServer(trajAct);
    sendGoalAndWait(trajAct,trajGoal);
    
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