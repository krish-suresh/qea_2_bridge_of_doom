function pose = getNeatoPosition(msg)
    for i = 1 : length(msg.Name)
        if strcmp(msg.Name{i}, 'neato_standalone')
            pose(1) = msg.Pose(i).Position.X;
            pose(2) = msg.Pose(i).Position.Y;
            posTheta = quat2eul([msg.Pose(i).Orientation.X, msg.Pose(i).Orientation.Y, msg.Pose(i).Orientation.Z, msg.Pose(i).Orientation.W]);
            pose(3) = posTheta(3);
            return
        end
    end
end