function plot_traj(q, robot, end_effector)
    num_frames = length(q');
    grip_position = zeros(num_frames,3);
    for k = 1:num_frames
        grip_position(k,:) = tform2trvec(getTransform(robot, q(:,k)', end_effector));
    end

    figure(1);
    show(robot);
    xlim('auto')
    ylim('auto')
    zlim('auto')
    camva('auto');

    hold on
    p = plot3(grip_position(1,1), grip_position(1,2), grip_position(1,3));

    hold on
    for k = 1:size(q',1)
        show(robot, q(:,k)', 'PreservePlot', false);
        p.XData(k) = grip_position(k,1);
        p.YData(k) = grip_position(k,2);
        p.ZData(k) = grip_position(k,3);
        pause(0.01)
    end
hold off

