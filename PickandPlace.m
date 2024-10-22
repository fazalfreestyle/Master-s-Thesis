% Load the URDF file and create a RigidBodyTree object
ned = importrobot("C:\Users\fazal\Downloads\ned2\ned2\niryo_ned2_gripper1_n_camera.urdf");

% Define end effector offset (if any)
eeoffset = 0;

% Create a rigid body for the end effector
eeBody = robotics.RigidBody("end_effector");
setFixedTransform(eeBody.Joint, trvec2tform([eeoffset, 0, 0]));

% Attach the end effector body to the robot model (ned)
addBody(ned, eeBody, "tool_link");

% Create an inverse kinematics object for the robot
ik = inverseKinematics("RigidBodyTree", ned);
weight = [0.1, 0.1, 0, 1, 1, 1];
initialguess = ned.homeConfiguration;

% ROS initialization
rosshutdown;
robot1_IP = 'http://192.168.11.190:11311';
user_IP = '192.168.11.115';
setenv('ROS_MASTER_URI',robot1_IP) %IP of the Ned
setenv('ROS_IP',user_IP) %IP of the computer
rosinit(robot1_IP);

% Subscribe to the current joint states
jointStateSub = rossubscriber("/niryo_robot_follow_joint_trajectory_controller/state");

% Define home position and work environment
home_position = [0.14, 0, 0.20];
home_orientation = [0, 0.77, 0];
work_position = [0, 0.415, 0.09];
work_orientation = [2.365, 1.540, 2.360];

% Define above object and object coordinates for each object
above_object_positions = {
    [0.28, 0.075, 0.11], [0.28, -0.135, 0.11], [0.11, -0.32, 0.11], ...
    [-0.09, -0.32, 0.11], [-0.28, -0.15, 0.11], [-0.28, 0.065, 0.11]
};
object_positions = {
    [0.28, 0.075, 0.09], [0.28, -0.135, 0.09], [0.11, -0.32, 0.09], ...
    [-0.09, -0.32, 0.09], [-0.28, -0.15, 0.09], [-0.28, 0.065, 0.09]
};
object_orientations = repmat([0.000, 1.540, 0.000], 6, 1);
openGripper();

% Start the sequence
disp('Moving to Home Position...');
move(ned, home_position, home_orientation, jointStateSub);

for i = 1:6
    % Move above the current object
    disp(['Moving above Object ', num2str(i)]);
    move(ned, above_object_positions{i}, object_orientations(i, :), jointStateSub);

    % Move to the object to grab it
    disp(['Moving to Object ', num2str(i)]);
    move(ned, object_positions{i}, object_orientations(i, :), jointStateSub);

    % Simulate grabbing the object
    disp(['Grabbing Object ', num2str(i)]);
    closeGripper();  % Use the closeGripper function
    pause(0.2);
    % Move back above the object
    disp(['Moving above Object ', num2str(i), ' after grabbing']);
    move(ned, above_object_positions{i}, object_orientations(i, :), jointStateSub);

    % Move to the work environment to drop the object
    disp(['Moving to Work Environment to drop Object ', num2str(i)]);
    move(ned, work_position, work_orientation, jointStateSub);

    % Simulate dropping the object
    disp(['Dropping Object ', num2str(i)]);
    openGripper();  % Use the openGripper function
    pause(0.2);
end

disp('All objects placed. Moving back to Home Position.');
move(ned, home_position, home_orientation, jointStateSub);

rosshutdown;

%% Function to move the robot to the given position and orientation
function move(ned, vect, eul, jointStateSub)
    ik = inverseKinematics("RigidBodyTree", ned);
    weight = [0.1 0.1 0 1 1 1];
    initialguess = ned.homeConfiguration;

    % Convert position and orientation to transformation matrix
    postform = trvec2tform(vect);
    eultform = eul2tform(eul, 'ZYX');
    tform = postform * eultform;

    % Solve inverse kinematics
    configSoln = ik("end_effector", tform, weight, initialguess);
    cell = struct2cell(configSoln);
    Joint = cell(2,:,:);
    matrixJoints = cell2mat(Joint);

    % Create ROS publishers and send the trajectory to the robot
    NedCmd = rospublisher("/niryo_robot_follow_joint_trajectory_controller/command");
    CmdMsg = rosmessage(NedCmd);
    CmdPoint = rosmessage('trajectory_msgs/JointTrajectoryPoint');

    % Fill in the joint positions
    for i = 1:6
        CmdPoint.Positions(i) = matrixJoints(i);
    end

    % Set velocities, accelerations, and time to complete the movement
    CmdPoint.Velocities = zeros(1, 6);
    CmdPoint.Accelerations = zeros(1, 6);
    CmdPoint.TimeFromStart = ros.msg.Duration(3);  % Duration of 3 seconds

    % Set the message header and joint names
    CmdMsg.Header.Stamp = rostime("now") + rosduration(0.05);
    CmdMsg.JointNames = {'joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6'};
    CmdMsg.Points = CmdPoint;

    % Debug: Show joint values being sent
    disp('Sending joint positions:');
    disp(matrixJoints);

    % Send the command to the robot
    send(NedCmd, CmdMsg);

    % Wait until the robot reaches the target position by comparing joint states
    maxWaitTime = 10;  % Timeout in seconds
    tolerance = 0.01;  % Tolerance for joint position comparison
    startTime = tic;

    while true
        % Get the current joint states from the subscriber
        currentState = receive(jointStateSub, 10);  % Timeout after 10 seconds
        currentJoints = currentState.Actual.Positions;

        % Compare current joint positions to the target joint positions
        if all(abs(currentJoints - matrixJoints) < tolerance)
            disp('Target position reached.');
            break;
        end

        % Check if the loop has run for too long
        if toc(startTime) > maxWaitTime
            disp('Timeout: Target not reached within allowed time.');
            break;
        end

        % Debug: Print current vs target positions
        disp('Current Joints:');
        disp(currentJoints);
        disp('Target Joints:');
        disp(matrixJoints);

        pause(0.1);  % Small pause to avoid overloading the system
    end
end

%% Function to open the gripper
function openGripper()
    NedCmd2 = rospublisher("/niryo_robot_tools_commander/action_server/goal");
    CmdMsg2 = rosmessage(NedCmd2);
    CmdGoal = rosmessage('niryo_robot_tools_commander/ToolGoal');

    CmdGoal.Cmd.ToolId = 11;
    CmdGoal.Cmd.CmdType = 1;  % Open the gripper
    CmdGoal.Cmd.MaxTorquePercentage = 100;
    CmdGoal.Cmd.HoldTorquePercentage = 100;
    CmdGoal.Cmd.Speed = 500;
    CmdGoal.Cmd.Activate = 1;

    CmdMsg2.Header.Stamp = rostime("now") + rosduration(0.05);
    CmdMsg2.GoalId.Stamp = rostime("now") + rosduration(0.05);
    CmdMsg2.GoalId.Id = "OPENGRIPPER";  % Open the gripper
    CmdMsg2.Goal = CmdGoal;

    send(NedCmd2, CmdMsg2);
    disp('Gripper opened.');
end

%% Function to close the gripper
function closeGripper()
    NedCmd2 = rospublisher("/niryo_robot_tools_commander/action_server/goal");
    CmdMsg2 = rosmessage(NedCmd2);
    CmdGoal = rosmessage('niryo_robot_tools_commander/ToolGoal');

    CmdGoal.Cmd.ToolId = 11;
    CmdGoal.Cmd.CmdType = 2;  % Close the gripper
    CmdGoal.Cmd.MaxTorquePercentage = 100;
    CmdGoal.Cmd.HoldTorquePercentage = 100;
    CmdGoal.Cmd.Speed = 500;
    CmdGoal.Cmd.Activate = 1;

    CmdMsg2.Header.Stamp = rostime("now") + rosduration(0.05);
    CmdMsg2.GoalId.Stamp = rostime("now") + rosduration(0.05);
    CmdMsg2.GoalId.Id = "CLOSEGRIPPER";  % Close the gripper
    CmdMsg2.Goal = CmdGoal;

    send(NedCmd2, CmdMsg2);
    disp('Gripper closed.');
end
