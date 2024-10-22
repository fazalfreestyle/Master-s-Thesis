function runRobot1Task(node1, objectIndex)
    % Load the URDF file and create a RigidBodyTree object for Robot 1
    ned1 = importrobot("C:\Users\fazal\Downloads\ned2\ned2\niryo_ned2_gripper1_n_camera.urdf");
    ik1 = inverseKinematics("RigidBodyTree", ned1);

    % Set up ROS publishers and subscribers for Robot 1
    jointStateSub1 = ros.Subscriber(node1, "/niryo_robot_follow_joint_trajectory_controller/state", 'control_msgs/JointTrajectoryControllerState');
    nedCmdPub1 = ros.Publisher(node1, "/niryo_robot_follow_joint_trajectory_controller/command", 'trajectory_msgs/JointTrajectory');
    gripperCmdPub1 = ros.Publisher(node1, '/niryo_robot_tools_commander/action_server/goal', 'niryo_robot_tools_commander/ToolActionGoal');
    
    % Initialize gripper command message
    CmdMsg1 = rosmessage(gripperCmdPub1);

    % Define positions for Robot 1
    home_position = [0.14, 0, 0.20];
    home_orientation = [0, 0.77, 0];
    work_position = [-0.015, 0.420, 0.09];  % Work environment
    work_orientation = [0, 1.540, 0];  % Work orientation
    above_work_position = [-0.015, 0.420, 0.11];  % 20mm above the work environment

    % Object positions and orientations (objectIndex from the main loop)
    above_object_positions = {
        [0.28, 0.075, 0.11], [0.28, -0.135, 0.11], [0.11, -0.32, 0.11], ...
        [-0.09, -0.32, 0.11], [-0.28, -0.15, 0.11], [-0.28, 0.065, 0.11]
    };
    object_positions = {
        [0.28, 0.075, 0.09], [0.28, -0.135, 0.09], [0.11, -0.32, 0.09], ...
        [-0.09, -0.32, 0.09], [-0.28, -0.15, 0.09], [-0.28, 0.065, 0.09]
    };
    object_orientations = repmat([0.000, 1.540, 0.000], 6, 1);  % All objects same orientation

    % Move to home position
    disp('Robot 1: Moving to Home Position...');
    moveRobot(ned1, ik1, home_position, home_orientation, jointStateSub1, nedCmdPub1);
    
    % Close gripper before starting
    disp('Robot 1: Closing gripper...');
    controlGripper(gripperCmdPub1, CmdMsg1, 2);  % Close gripper

    % Move above the object
    disp(['Robot 1: Moving above Object ', num2str(objectIndex)]);
    moveRobot(ned1, ik1, above_object_positions{objectIndex}, object_orientations(objectIndex, :), jointStateSub1, nedCmdPub1);

    % Open the gripper above the object
    disp(['Robot 1: Opening gripper above Object ', num2str(objectIndex)]);
    controlGripper(gripperCmdPub1, CmdMsg1, 1);  % Open gripper

    % Move to the object and grab it
    disp(['Robot 1: Moving to Object ', num2str(objectIndex)]);
    moveRobot(ned1, ik1, object_positions{objectIndex}, object_orientations(objectIndex, :), jointStateSub1, nedCmdPub1);
    
    % Close the gripper after grabbing the object
    disp('Robot 1: Closing gripper after grabbing the object...');
    controlGripper(gripperCmdPub1, CmdMsg1, 2);  % Close gripper

    % Move back above the object after grabbing it
    disp(['Robot 1: Moving above Object ', num2str(objectIndex), ' after grabbing']);
    moveRobot(ned1, ik1, above_object_positions{objectIndex}, object_orientations(objectIndex, :), jointStateSub1, nedCmdPub1);

    % Move above the work environment
    disp('Robot 1: Moving above the Work Environment');
    moveRobot(ned1, ik1, above_work_position, work_orientation, jointStateSub1, nedCmdPub1);

    % Move to the work environment to drop the object
    disp(['Robot 1: Moving to Work Environment to drop Object ', num2str(objectIndex)]);
    moveRobot(ned1, ik1, work_position, work_orientation, jointStateSub1, nedCmdPub1);

    % Open the gripper to drop the object
    disp(['Robot 1: Opening gripper to drop Object ', num2str(objectIndex)]);
    controlGripper(gripperCmdPub1, CmdMsg1, 1);  % Open gripper

    % Move back above the work environment
    disp('Robot 1: Moving above the Work Environment after dropping');
    moveRobot(ned1, ik1, above_work_position, work_orientation, jointStateSub1, nedCmdPub1);

    % Move back to home position
    disp('Robot 1: Moving back to Home Position.');
    moveRobot(ned1, ik1, home_position, home_orientation, jointStateSub1, nedCmdPub1);

    disp(['Robot 1: Task completed for Object ', num2str(objectIndex)]);
end
