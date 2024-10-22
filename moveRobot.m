function moveRobot(ned, ik, vect, eul, jointStateSub, nedCmdPub)
    weight = [0.1 0.1 0 1 1 1];
    initialguess = ned.homeConfiguration;

    % Convert position and orientation to transformation matrix
    postform = trvec2tform(vect);
    eultform = eul2tform(eul, 'ZYX');
    tform = postform * eultform;

    % Solve inverse kinematics
    configSoln = ik("tool_link", tform, weight, initialguess);
    cellData = struct2cell(configSoln);
    matrixJoints = cell2mat(cellData(2,:,:));

    % Extract only the first 6 joints
    matrixJoints = matrixJoints(1:6);

    % Create ROS message for the command
    CmdMsg = rosmessage(nedCmdPub);
    CmdPoint = rosmessage('trajectory_msgs/JointTrajectoryPoint');

    % Fill in the joint positions
    CmdPoint.Positions = matrixJoints;

    % Set velocities, accelerations, and time to complete the movement
    CmdPoint.Velocities = zeros(1, length(matrixJoints));
    CmdPoint.Accelerations = zeros(1, length(matrixJoints));
    CmdPoint.TimeFromStart = ros.msg.Duration(10);  % Set movement time

    % Set the message header and joint names
    CmdMsg.Header.Stamp = rostime("now") + rosduration(0.05);
    CmdMsg.JointNames = {'joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6'};
    CmdMsg.Points = CmdPoint;

    % Send the command to the robot
    send(nedCmdPub, CmdMsg);

    % Wait for the robot to reach the target position
    maxWaitTime = 30;  % Timeout in seconds
    tolerance = 0.01;  % Tolerance for joint position comparison
    startTime = tic;

    while true
        % Get the current joint states
        currentState = receive(jointStateSub, 10);  % Timeout after 10 seconds
        currentJoints = currentState.Actual.Positions;

        % Debug information
        % disp(['Current State - ', mat2str(currentJoints)]);
        % disp(['matrixJoints - ', mat2str(matrixJoints(:))]);

        % Ensure currentJoints and matrixJoints are compatible in size
        if length(currentJoints) == length(matrixJoints)
            toleranceValues = abs(currentJoints - matrixJoints(:));
            % disp(['tolerance - ', mat2str(toleranceValues')]);  % Display the tolerance

            if all(toleranceValues < tolerance)
                disp('Target position reached.');
                break;
            end
        else
            disp('Error: currentJoints and matrixJoints sizes do not match.');
        end

        % Check if timeout has occurred
        if toc(startTime) > maxWaitTime
            disp('Timeout: Target not reached within allowed time.');
            break;
        end

        pause(0.1);  % Small pause to avoid overloading the system
    end
end
