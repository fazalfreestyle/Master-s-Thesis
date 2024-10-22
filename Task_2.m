% Define the URDF model for Niryo Ned 2
ned = importrobot("C:\ned_ros\niryo_robot_description\urdf\ned2\niryo_ned2_gripper1_n_camera.urdf");

% Add the end effector to the robot model
eeoffset = 0;
eeBody = robotics.RigidBody("end_effector");
setFixedTransform(eeBody.Joint, trvec2tform([eeoffset, 0, 0]));
addBody(ned, eeBody, "tool_link");

% Inverse kinematics for the position and orientation
ik = inverseKinematics("RigidBodyTree", ned);
weight = [0.1, 0.1, 0, 1, 1, 1];
initialguess = ned.homeConfiguration;

% Define target pose (position and orientation)
pose_M = [0.000, -0.415, 0.100];  % Target position [x, y, z]
orientation = [-1.540, 1.540, 0.000];  % Target orientation (roll, pitch, yaw)

% pose_M = [0.150, 0.000, 0.200];  % Target position [x, y, z]
% orientation = [0, 0.77, 0];  % Target orientation (roll, pitch, yaw)
% Convert position and orientation to a transformation matrix
tform_pos = trvec2tform(pose_M);  % Create translation matrix from position
tform_orient = eul2tform(orientation, 'ZYX');  % Create rotation matrix from orientation

% Combine position and orientation into a full transformation matrix
tform = tform_pos * tform_orient;

% Solve inverse kinematics to get joint positions
configSoln = ik("end_effector", tform, weight, initialguess);
cell = struct2cell(configSoln);
Joint = cell(2,:,:);
matrixJoints = cell2mat(Joint);

% ROS Initialization (adjust as needed for ROS 2 bridge)
rosshutdown;  % Shutdown any existing ROS connection

% Connect to Niryo Ned 2 (using ROS bridge)
robot1_IP = 'http://192.168.130.152:11311';  % ROS 1 master from ROS 2 bridge
rosinit(robot1_IP);

% Subscribe and publish commands to the Niryo controller
NedState = rossubscriber("/niryo_robot_follow_joint_trajectory_controller/state");
NedCmd = rospublisher("/niryo_robot_follow_joint_trajectory_controller/command");
CmdMsg = rosmessage(NedCmd);
CmdPoint = rosmessage('trajectory_msgs/JointTrajectoryPoint');

% Set the calculated joint positions
for i = 1:6
    CmdPoint.Positions(i) = matrixJoints(i);
end

% Configure movement timing and execution
CmdPoint.Velocities = zeros(1, 6);
CmdPoint.Accelerations = zeros(1, 6);
CmdPoint.TimeFromStart = ros.msg.Duration(3);  % Move over 3 seconds
CmdMsg.Header.Stamp = rostime("now") + rosduration(0.05);
CmdMsg.JointNames = {'joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6'};
CmdMsg.Points = CmdPoint;

% Send the command to move the robot
send(NedCmd, CmdMsg);

% Wait for the robot to reach the position (optional monitoring of joint state)
disp('Robot is moving to the target position and orientation...');
