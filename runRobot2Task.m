function runRobot2Task(node2, net)
    % Load the URDF file and create a RigidBodyTree object for Robot 2
    ned2 = importrobot("C:\Users\fazal\Downloads\ned2\ned2\niryo_ned2_gripper1_n_camera.urdf");
    ik2 = inverseKinematics("RigidBodyTree", ned2);

    % Set up ROS publishers and subscribers for Robot 2
    jointStateSub2 = ros.Subscriber(node2, "/niryo_robot_follow_joint_trajectory_controller/state", 'control_msgs/JointTrajectoryControllerState');
    nedCmdPub2 = ros.Publisher(node2, "/niryo_robot_follow_joint_trajectory_controller/command", 'trajectory_msgs/JointTrajectory');
    gripperCmdPub2 = ros.Publisher(node2, '/niryo_robot_tools_commander/action_server/goal', 'niryo_robot_tools_commander/ToolActionGoal');
    cameraSub = ros.Subscriber(node2, "/niryo_robot_vision/compressed_video_stream", 'sensor_msgs/CompressedImage');

    % Create a message for the gripper
    CmdMsg2 = rosmessage(gripperCmdPub2);

    % Define positions and orientations for Robot 2
    home_position = [0.150, 0, 0.200];
    home_orientation = [0, 0.77, 0];
    work_pov_position = [0, -0.310, 0.150];
    work_above_object_position = [-0.005, -0.420, 0.150];
    work_object_position = [-0.005, -0.420, 0.100];
    cuboid_box_position = [-0.100, 0.300, 0.160];
    cylinder_box_position = [0.100, 0.300, 0.160];
    orientation = [-1.540, 1.540, 0];

    % Move to home position
    disp('Robot 2: Moving to Home Position...');
    moveRobot(ned2, ik2, home_position, home_orientation, jointStateSub2, nedCmdPub2);

    % Move to work environment for camera
    disp('Robot 2: Moving to Work Environment POV Camera Position...');
    moveRobot(ned2, ik2, work_pov_position, orientation, jointStateSub2, nedCmdPub2);

    % Ensure the camera feed is active before capturing
    cameraActiveSub = ros.Subscriber(node2, "/niryo_robot_vision/video_stream_is_active", 'std_msgs/Bool');
    cameraActiveMsg = receive(cameraActiveSub, 10);
    if cameraActiveMsg.Data
        % Capture the image from the camera
        disp('Capturing image...');
        try
            img = captureImage(cameraSub);  % Use the helper function to capture the image

            % Resize the image to match the network's input size
            imgResized = imresize(img, [224, 224]);  % Resize to 224x224 for the network

            % Classify the image using the pre-trained network
            predictedLabel = classify(net, imgResized);

            % Display the image with the predicted label for debugging
            imshow(img);
            title(['Predicted Shape: ', char(predictedLabel)]);  % Overlay label on the image
            disp(['Predicted Shape: ', char(predictedLabel)]);  % Display label in console
            drawnow;
            
        catch ME
            disp(['Error capturing image: ', ME.message]);
            error('Could not retrieve camera feed. Ensure the camera is active and connected.');
        end
    else
        error('Camera feed is not active.');
    end

    % Move above the object
    disp('Robot 2: Moving above the object...');
    moveRobot(ned2, ik2, work_above_object_position, orientation, jointStateSub2, nedCmdPub2);

    % Move to the object to grab it
    disp('Robot 2: Moving to object and grabbing...');
    moveRobot(ned2, ik2, work_object_position, orientation, jointStateSub2, nedCmdPub2);
    controlGripper(gripperCmdPub2, CmdMsg2, 2);  % Close gripper to grab the object

    % ** Debugging condition: Display exact comparison **
    disp(['Comparing predicted label: ', char(predictedLabel)]);
    
    % Move based on the predicted label
    if strcmp(char(predictedLabel), 'Cylinder')
        disp('Robot 2: Moving to Cylinder Box...');
        moveRobot(ned2, ik2, cylinder_box_position, orientation, jointStateSub2, nedCmdPub2);
    else
        disp('Robot 2: Moving to Cuboid Box...');
        moveRobot(ned2, ik2, cuboid_box_position, orientation, jointStateSub2, nedCmdPub2);
    end

    % Drop the object and move back to home
    controlGripper(gripperCmdPub2, CmdMsg2, 1);  % Open gripper to drop
    moveRobot(ned2, ik2, home_position, home_orientation, jointStateSub2, nedCmdPub2);

    disp('Robot 2: Task completed.');
end
