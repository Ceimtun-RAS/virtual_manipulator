function [q,qd,qdd,traj_times] = compute_trajectory(current_joints_Config, final_pose, robot, endEffector, traj_duration)
        time_step = 0.1;
        ws_height = 0.1;
        ik = inverseKinematics('RigidBodyTree',robot);
        ik.SolverParameters.AllowRandomRestart = false;
        weights = [1 1 1 1 1 1];

        %Initial task config
        joint_init = wrapToPi(current_joints_Config');
        % Initial end-effector pose
        T0 = getTransform(robot, joint_init, endEffector);
        q0 = tform2trvec(T0);
        qF = tform2trvec(final_pose);

        % Time intervals
        traj_times = 0:time_step:traj_duration;
        
        % Retrieve task configurations between initial and final
        wpts = zeros(3,5);
        wpts(:,1) = q0';
        wpts(:,2) = q0';
        if q0(3) < ws_height
            wpts(3,2) = ws_height;
        end
        wpts(:,3) = (qF+q0)/2;
        wpts(3,3) = 2*ws_height;
        wpts(:,4) = qF;
        if qF(3) < ws_height
            wpts(3,4) = ws_height;
        end
        wpts(:,5) = qF;   
        tpts = [0 traj_duration/8 traj_duration/2 7*traj_duration/8 traj_duration];

        
        [q,~,~,~] = cubicpolytraj(wpts,tpts,traj_times);
        % [q,qd,qdd,pp] = cubicpolytraj(wayPoints,timePoints,tSamples)
        ang0 = tform2eul(T0,'XYZ');
        angF = tform2eul(final_pose,'XYZ');
        ang = wrapToPi([ang0;angF]);
        t_ang = [0 traj_duration];
        R = wrapToPi(interp1(t_ang,ang(:,1),traj_times));
        P = wrapToPi(interp1(t_ang,ang(:,2),traj_times));
        Y = wrapToPi(interp1(t_ang,ang(:,3),traj_times));
        % Compute corresponding joint configurations
        robotPos = zeros(length(traj_times),numel(joint_init));
        initialGuess = wrapToPi(joint_init);
        for i=1:length(traj_times)
            T = trvec2tform(q(:,i)')*eul2tform([R(i) P(i) Y(i)],'XYZ');
            robotPos(i,:) = ik(endEffector,T,weights,initialGuess);
            robotPos(i,:) = wrapToPi(robotPos(i,:));
            initialGuess = robotPos(i,:);            
        end   

        %%  Compute joint velocities and accelerations at required rate for execution by the robot
        % disp('Done planning trajectory, now sampling...')

        % Interpolated joint velocities
        h = time_step;
        robotVelTemp = diff(robotPos)/h;
        robotVel= [zeros(1,numel(joint_init));robotVelTemp];

        % Interpolated joint accelerations
        robotAccTemp = diff(robotVelTemp)/h;
        robotAcc = [zeros(2,numel(joint_init));robotAccTemp];

        q = robotPos';
        qd = robotVel';
        qdd = robotAcc';    
        

end
