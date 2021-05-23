function drop_item(bin_name)
    if bin_name == "blue"
        bin_pos = [-0.3 -0.4 0.4];
    elseif bin_name == "green"
        bin_pos = [-0.3 0.4 0.4];
    end
    
    move_time = 4;

    jointSub = rossubscriber('/my_gen3/joint_states');
    ros_action='/my_gen3/gen3_joint_trajectory_controller/follow_joint_trajectory';
    [trajAct,trajGoal] = rosactionclient( ros_action);
    showdetails(trajGoal)

    orientation = [pi 0 pi/2];
    H_transform = trvec2tform(bin_pos)*eul2tform(orientation,'XYZ');

    jointMsg = receive(jointSub,2);
    jointPosition =  jointMsg.Position(2:8);

    jointMsg.Name;

    [q,qd,qdd,trajTimes] = RoboCupManipulation_computeTrajectory( ...
                            jointPosition, H_transform, robot, 'gripper', move_time);                      

    trajGoal = packageJointTrajectory(trajGoal,q,qd,qdd,trajTimes);

    waitForServer(trajAct);
    sendGoal(trajAct,trajGoal)
    waitForServer(trajAct);
    SLActivateGripper("open")
end