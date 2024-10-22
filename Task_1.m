% Shut down any existing ROS connections
rosshutdown; 

% Initialize ROS for Robot 1
robot1_IP = '192.168.11.190';  % IP address of Robot 1
rosinit(robot1_IP);  % Connect to Robot 1's ROS master

% Subscribe to the correct topic for Robot 1 with the correct message type
jointStateSub1 = rossubscriber('/niryo_robot_follow_joint_trajectory_controller/state', 'control_msgs/JointTrajectoryControllerState');
pause(1);  % Wait for the subscription to initialize

% Receive and display the current joint states of Robot 1
jointState1 = receive(jointStateSub1, 10);  % Timeout after 10 seconds if no message is received
disp('Robot 1 Joint States:');
disp(jointState1.Actual.Positions);  % Display the joint positions

% Once done with Robot 1, shut down the ROS connection
rosshutdown;

% Initialize ROS for Robot 2
robot2_IP = '192.168.11.152';  % IP address of Robot 2
rosinit(robot2_IP);  % Connect to Robot 2's ROS master

% Subscribe to the correct topic for Robot 2 with the correct message type
jointStateSub2 = rossubscriber('/niryo_robot_follow_joint_trajectory_controller/state', 'control_msgs/JointTrajectoryControllerState');
pause(1);  % Wait for the subscription to initialize

% Receive and display the current joint states of Robot 2
jointState2 = receive(jointStateSub2, 10);  % Timeout after 10 seconds if no message is received
disp('Robot 2 Joint States:');
disp(jointState2.Actual.Positions);  % Display the joint positions

% Shut down ROS after communicating with Robot 2
rosshutdown;
