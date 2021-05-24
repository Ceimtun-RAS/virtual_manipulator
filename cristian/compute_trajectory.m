function [q,qd,qdd,trajTimes] = compute_trajectory(current_joints_Config, final_pose, robot, endEffector, traj_duration)
        timestep = 0.1;
        ik = inverseKinematics('RigidBodyTree',robot);
        ik.SolverParameters.AllowRandomRestart = false;
        weights = [1 1 1 1 1 1];

        %Initial task config
        jointInit = wrapToPi(current_joints_Config');
        % Initial end-effector pose
        taskInit = getTransform(robot, jointInit, endEffector);

        % Time intervals
        timeInterval = [0;traj_duration];
        trajTimes = timeInterval(1):timestep:timeInterval(end);

        % Retrieve task configurations between initial and final
        [s,sd,sdd] = trapveltraj(timeInterval',numel(trajTimes)); % [q,qd,qdd,tSamples,pp] = trapveltraj(wayPoints,numSamples)
        [T, ~, ~] = transformtraj(taskInit,final_pose,timeInterval,trajTimes, 'TimeScaling',[s;sd;sdd]/timeInterval(end));

        % Compute corresponding joint configurations
        robotPos = zeros(size(T,3),numel(jointInit));
        initialGuess = wrapToPi(jointInit);
        for i=1:size(T,3)            
            robotPos(i,:) = ik(endEffector,T(:,:,i),weights,initialGuess);
            robotPos(i,:) = wrapToPi(robotPos(i,:));
            initialGuess = robotPos(i,:);            
        end   

        %%  Compute joint velocities and accelerations at required rate for execution by the robot
        % disp('Done planning trajectory, now sampling...')

        % Interpolated joint velocities
        h = timestep;
        robotVelTemp = diff(robotPos)/h;
        robotVel= [zeros(1,numel(jointInit));robotVelTemp];

        % Interpolated joint accelerations
        robotAccTemp = diff(robotVelTemp)/h;
        robotAcc = [zeros(2,numel(jointInit));robotAccTemp];

        q = robotPos';
        qd = robotVel';
        qdd = robotAcc';    
        

end
