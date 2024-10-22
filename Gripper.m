% % Initialize ROS if not already initialized
rosshutdown;
robot1_IP = 'http://192.168.130.190:11311';  % IP of the Ned
user_IP = '192.168.130.115';  % IP of your computer
setenv('ROS_MASTER_URI', robot1_IP);  % Set ROS master URI
setenv('ROS_IP', user_IP);  % Set ROS IP
rosinit(robot1_IP);  % Initialize ROS
% 
% % Create the gripper publisher and message
NedCmd2 = rospublisher("/niryo_robot_tools_commander/action_server/goal");
CmdMsg2 = rosmessage(NedCmd2);

% Test the gripper functions
controlGripper(NedCmd2, CmdMsg2, 1);  % Open the gripper
pause(2);  % Wait for 2 seconds
controlGripper(NedCmd2, CmdMsg2, 2);  % Close the gripper

% Shutdown ROS after use
% rosshutdown;

% Define a function to control the gripper
function controlGripper(NedCmd2, CmdMsg2, action)
    % Create a gripper command message
    CmdGoal = rosmessage('niryo_robot_tools_commander/ToolGoal');
    
    % Set Tool ID and Command Type
    CmdGoal.Cmd.ToolId = 11;  % Gripper tool ID
    CmdGoal.Cmd.CmdType = action;  % 1 for open, 2 for close
    
    % Set correct values for speed, torque, and activation
    CmdGoal.Cmd.Speed = 500;  % Set the speed (0-500 recommended)
    CmdGoal.Cmd.MaxTorquePercentage = 100;  % Set maximum torque to 100%
    CmdGoal.Cmd.HoldTorquePercentage = 100;  % Set hold torque to 100%
    CmdGoal.Cmd.Activate = 1;  % Activate the command

    % Set unique Goal ID for each command
    CmdMsg2.Header.Stamp = rostime("now") + rosduration(0.05);
    CmdMsg2.GoalId.Stamp = rostime("now") + rosduration(0.05);
    
    % Set unique IDs for opening or closing the gripper
    if action == 1
        CmdMsg2.GoalId.Id = "OPENGRIPPER";
        disp('Opening the gripper...');
    else
        CmdMsg2.GoalId.Id = "CLOSEGRIPPER";
        disp('Closing the gripper...');
    end
    
    % Assign the goal to the message
    CmdMsg2.Goal = CmdGoal;
    
    % Send the message to execute the action
    send(NedCmd2, CmdMsg2);
end
