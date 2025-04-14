classdef MultiCobotController < handle
    %MULTICOBOTCONTROLLER Manages two Niryo NED robots, each with separate logging
    %
    % EXAMPLE USAGE:
    %   1. Initialize controller
    %       controller = MultiCobotController('192.168.0.101', '192.168.0.102', ...
    %           'path/to/robot1.urdf', 'path/to/robot2.urdf', 'path/to/trainedNet.mat');
    %
    %   2. Record separated logs
    %       controller.startRecording('exp1');
    %       controller.runRobot1Task(1);
    %       controller.runRobot2Task(1);
    %       controller.stopRecording();
    %
    %   3. Analyze results
    %       controller.analyzeRobot1Performance('exp1');
    %       controller.analyzeRobot2Performance('exp1');
    %       controller.plotTaskCorrelation('exp1');  % cross-robot correlation
    %
    %   4. Display combined logs
    %       controller.showLogs('exp1');  % or whichever baseFileName you used
    %
    % ---------------------------------------------------------------------
    % This class provides:
    %   * Robot1 + Robot2 nodes, each with separate topics & logging
    %   * Per-robot .bag files with namespaced topics (/robot1, /robot2)
    %   * Robot-specific analysis & visualization
    %   * Optional cross-robot correlation analysis
    %

    properties (Access = public)
        % Robot 1 node, pubs, subs
        node1
        jointStateSub1
        nedCmdPub1
        gripperCmdPub1
        CmdMsg1
        gripperStatusSub1

        % Robot 2 node, pubs, subs
        node2
        jointStateSub2
        nedCmdPub2
        gripperCmdPub2
        CmdMsg2
        gripperStatusSub2
        cameraSub2

        % Logging subscribers
        jointStateLogger1
        jointStateLogger2
        gripperStatusLogger1
        gripperStatusLogger2
        cameraLogger2
        jointCommandLogger1
        jointCommandLogger2
        gripperCommandLogger1
        gripperCommandLogger2

        % URDF-based robot model, IK, etc.
        nedModel1
        nedModel2
        ik1
        ik2

        % ROS bag writer & logging
        bagWriter1
        bagWriter2
        isRecording = false

        % Classification network
        net
    end

    methods
        %% --- Constructor ---
        function obj = MultiCobotController(node1IP, node2IP, modelPath1, modelPath2, netPath)
            if ~ros.internal.Global.isNodeActive
                rosshutdown;
                rosinit('http://localhost:11311')  % Connect to default ROS master if none is active
            end

            % Initialize ROS Nodes
            obj.node1 = ros.Node('/robot1control', node1IP);
            obj.node2 = ros.Node('/robot2control', node2IP);

            % Load Robot 1 URDF Model
            obj.nedModel1 = importrobot(modelPath1);
            eeoffset = 0;
            eeBody = robotics.RigidBody("end_effector");
            eeJoint = robotics.Joint("ee_fixed","fixed");
            setFixedTransform(eeJoint, trvec2tform([eeoffset,0,0]));
            addBody(obj.nedModel1, eeBody, "tool_link");
            obj.ik1 = inverseKinematics("RigidBodyTree", obj.nedModel1);

            % Load Robot 2 URDF Model
            obj.nedModel2 = importrobot(modelPath2);
            eeBody2 = robotics.RigidBody("end_effector");
            eeJoint2 = robotics.Joint("ee_fixed","fixed");
            setFixedTransform(eeJoint2, trvec2tform([eeoffset,0,0]));
            addBody(obj.nedModel2, eeBody2, "tool_link");
            obj.ik2 = inverseKinematics("RigidBodyTree", obj.nedModel2);

            % Initialize sensor/state subscribers
            obj.jointStateSub1 = ros.Subscriber(obj.node1, ...
                "/niryo_robot_follow_joint_trajectory_controller/state", ...
                'control_msgs/JointTrajectoryControllerState');

            obj.jointStateSub2 = ros.Subscriber(obj.node2, ...
                "/niryo_robot_follow_joint_trajectory_controller/state", ...
                'control_msgs/JointTrajectoryControllerState');

            obj.gripperStatusSub1 = ros.Subscriber(obj.node1, ...
                "/niryo_robot_tools_commander/action_server/status", ...
                "actionlib_msgs/GoalStatusArray");

            obj.gripperStatusSub2 = ros.Subscriber(obj.node2, ...
                "/niryo_robot_tools_commander/action_server/status", ...
                "actionlib_msgs/GoalStatusArray");

            obj.cameraSub2 = ros.Subscriber(obj.node2, ...
                "/niryo_robot_vision/compressed_video_stream", ...
                'sensor_msgs/CompressedImage');

            % Initialize command publishers
            obj.initCommandPublishers();

            % Initialize separate logging subscribers for each robot
            obj.initLoggingSubscribers();

            % Load optional classification network
            obj.loadNetwork(netPath);

            disp('MultiCobotController initialized successfully.');
        end

        %% --- Initialize Logging Subscribers ---
        function initLoggingSubscribers(obj)
            % Robot 1 logging
            obj.jointStateLogger1 = ros.Subscriber(obj.node1, ...
                "/niryo_robot_follow_joint_trajectory_controller/state", ...
                'control_msgs/JointTrajectoryControllerState', ...
                @(src,msg) obj.recordCallback(msg, 'robot1', obj.jointStateSub1.TopicName));

            obj.jointCommandLogger1 = ros.Subscriber(obj.node1, ...
                "/niryo_robot_follow_joint_trajectory_controller/command", ...
                "trajectory_msgs/JointTrajectory", ...
                @(src, msg) obj.recordCallback(msg, 'robot1', obj.nedCmdPub1.TopicName));

            obj.gripperCommandLogger1 = ros.Subscriber(obj.node1, ...
                "/niryo_robot_tools_commander/action_server/goal", ...
                "niryo_robot_tools_commander/ToolActionGoal", ...
                @(src, msg) obj.recordCallback(msg, 'robot1', obj.gripperCmdPub1.TopicName));

            obj.gripperStatusLogger1 = ros.Subscriber(obj.node1, ...
                "/niryo_robot_tools_commander/action_server/status", ...
                "actionlib_msgs/GoalStatusArray", ...
                @(src, msg) obj.recordCallback(msg, 'robot1', obj.gripperStatusSub1.TopicName));

            % Robot 2 logging
            obj.jointStateLogger2 = ros.Subscriber(obj.node2, ...
                "/niryo_robot_follow_joint_trajectory_controller/state", ...
                'control_msgs/JointTrajectoryControllerState', ...
                @(src, msg) obj.recordCallback(msg, 'robot2', obj.jointStateSub2.TopicName));

            obj.jointCommandLogger2 = ros.Subscriber(obj.node2, ...
                "/niryo_robot_follow_joint_trajectory_controller/command", ...
                "trajectory_msgs/JointTrajectory", ...
                @(src, msg) obj.recordCallback(msg, 'robot2', obj.nedCmdPub2.TopicName));

            obj.gripperCommandLogger2 = ros.Subscriber(obj.node2, ...
                "/niryo_robot_tools_commander/action_server/goal", ...
                "niryo_robot_tools_commander/ToolActionGoal", ...
                @(src, msg) obj.recordCallback(msg, 'robot2', obj.gripperCmdPub2.TopicName));

            obj.gripperStatusLogger2 = ros.Subscriber(obj.node2, ...
                "/niryo_robot_tools_commander/action_server/status", ...
                "actionlib_msgs/GoalStatusArray", ...
                @(src, msg) obj.recordCallback(msg, 'robot2', obj.gripperStatusSub2.TopicName));

            obj.cameraLogger2 = ros.Subscriber(obj.node2, ...
                "/niryo_robot_vision/compressed_video_stream", ...
                'sensor_msgs/CompressedImage', ...
                @(src, msg) obj.recordCallback(msg, 'robot2', obj.cameraSub2.TopicName));
        end

        %% --- Initialize Command Publishers ---
        function initCommandPublishers(obj)
            % Robot 1 Pubs
            obj.nedCmdPub1 = ros.Publisher(obj.node1, ...
                "/niryo_robot_follow_joint_trajectory_controller/command", ...
                "trajectory_msgs/JointTrajectory");
            obj.gripperCmdPub1 = ros.Publisher(obj.node1, ...
                "/niryo_robot_tools_commander/action_server/goal", ...
                "niryo_robot_tools_commander/ToolActionGoal");
            obj.CmdMsg1 = rosmessage(obj.gripperCmdPub1);

            % Robot 2 Pubs
            obj.nedCmdPub2 = ros.Publisher(obj.node2, ...
                "/niryo_robot_follow_joint_trajectory_controller/command", ...
                "trajectory_msgs/JointTrajectory");
            obj.gripperCmdPub2 = ros.Publisher(obj.node2, ...
                "/niryo_robot_tools_commander/action_server/goal", ...
                "niryo_robot_tools_commander/ToolActionGoal");
            obj.CmdMsg2 = rosmessage(obj.gripperCmdPub2);
        end

        %% --- Load Classification Network (Optional) ---
        function loadNetwork(obj, netPath)
            if exist(netPath, 'file')
                loadedNet = load(netPath);
                if isfield(loadedNet, 'trainedNet')
                    obj.net = loadedNet.trainedNet;
                elseif isfield(loadedNet, 'net')
                    obj.net = loadedNet.net;
                else
                    error('No valid network found in the .mat file.');
                end
            else
                warning('No network model loaded. Provide a valid netPath if needed.');
            end
        end

        %% --- Logging Callback with Validation ---
        function recordCallback(obj, msg, robotID, topicName)
            % Validate robot ID
            if ~any(strcmp(robotID, {'robot1', 'robot2'}))
                error('Invalid robotID: %s', robotID);
            end
        
            % Only log if recording is active
            if obj.isRecording
                % Use the actual topic name (not namespaced aliases)
                switch robotID
                    case 'robot1'
                        write(obj.bagWriter1, topicName, rostime("now"), msg);
%                         disp(['✅ [Robot1] Logged: ', topicName]);
                    case 'robot2'
                        write(obj.bagWriter2, topicName, rostime("now"), msg);
%                         disp(['✅ [Robot2] Logged: ', topicName]);
                end
            end
        end


        %% --- Start Recording (with bag management checks) ---
        function startRecording(obj, baseFileName)
            % Validate filename
            if ~isvarname(baseFileName)
                error('Invalid base filename: %s', baseFileName);
            end

            % Prevent accidental overwrites
            robot1Bag = strcat(baseFileName, '_robot1.bag');
            robot2Bag = strcat(baseFileName, '_robot2.bag');
            if exist(robot1Bag, 'file')
                error('Robot1 bag file already exists: %s', robot1Bag);
            end
            if exist(robot2Bag, 'file')
                error('Robot2 bag file already exists: %s', robot2Bag);
            end

            % Create bag writers
            obj.bagWriter1 = rosbagwriter(robot1Bag);
            obj.bagWriter2 = rosbagwriter(robot2Bag);
            obj.isRecording = true;
            disp('Recording started.');

            % Record the initial states from each robot
            obj.recordInitialState('robot1');
            obj.recordInitialState('robot2');
        end

        %% --- Stop Recording ---
        function stopRecording(obj)
            if obj.isRecording
                delete(obj.bagWriter1);
                delete(obj.bagWriter2);
                obj.isRecording = false;
%                 disp('Recording stopped at: %s\n', datestr(now, 'HH:MM:SS'));
            else
                disp('Not currently recording; nothing to stop.');
            end
        end

        %% --- Record Initial States (with validity checks) ---
        function recordInitialState(obj, robotID)
            % Only record messages newer than 5s before "now"
            minValidTime = rostime('now') - rosduration(5);

            switch robotID
                case 'robot1'
                    subs = {
                        obj.jointStateLogger1,       'joint_states';
                        obj.jointCommandLogger1,     'joint_commands';
                        obj.gripperCommandLogger1,   'gripper_commands';
                        obj.gripperStatusLogger1,    'gripper_status'
                    };
                case 'robot2'
                    subs = {
                        obj.jointStateLogger2,       'joint_states';
                        obj.jointCommandLogger2,     'joint_commands';
                        obj.gripperCommandLogger2,   'gripper_commands';
                        obj.gripperStatusLogger2,    'gripper_status';
                        obj.cameraLogger2,           'camera'
                    };
                otherwise
                    error('Unknown robotID: %s', robotID);
            end

            % Check each subscriber's latest message
            for i = 1:size(subs,1)
                subObj   = subs{i,1};
                topicStr = subs{i,2};
                latestMsg = subObj.LatestMessage;

                if ~isempty(latestMsg) && isfield(latestMsg,'Header') ...
                        && (latestMsg.Header.Stamp > minValidTime)
                    % Record if it meets the time threshold
                    obj.recordCallback(latestMsg, robotID, topicStr);
                end
            end
        end

        %% --- Robot 1 Task (Pick & Place) ---
        function runRobot1Task(obj, objectIndex)
            home_position = [0.14, 0, 0.20];
            home_orientation = [0, 0.77, 0];

            above_object_positions = {
                [0.200, -0.250, 0.110], ...
                [0.200, -0.150, 0.110], ...
                [0.200, -0.050, 0.110], ...
                [0.200,  0.050, 0.110], ...
                [0.200,  0.150, 0.110], ...
                [0.200,  0.250, 0.110], ...
                home_position
            };

            object_positions = {
                [0.200, -0.250, 0.090], ...
                [0.200, -0.150, 0.090], ...
                [0.200, -0.050, 0.090], ...
                [0.200,  0.050, 0.090], ...
                [0.200,  0.150, 0.090], ...
                [0.200,  0.250, 0.090]
            };

            object_orientations = {
                [0.000, 1.540, 0], ...
                [0.000, 1.540, 0], ...
                [0.000, 1.540, 0], ...
                [0.000, 1.540, 0], ...
                [0.000, 1.540, 0], ...
                [0.000, 1.540, 0], ...
                home_orientation
            };

            work_position = [-0.015, 0.420, 0.09];   % Work environment
            work_orientation = [0, 1.540, 0];
            above_work_position = [-0.015, 0.420, 0.11]; % 20mm above the work environment

            % If it's the first object, move to home and open gripper
            if objectIndex == 1
                obj.moveRobot(obj.nedModel1, obj.ik1, home_position, home_orientation, ...
                    obj.jointStateSub1, obj.nedCmdPub1);
                obj.controlGripper(obj.gripperCmdPub1, obj.CmdMsg1, 1, obj.gripperStatusSub1); % Open
            end

            % Start picking
            robot1Pick = tic;

            % Move above the chosen object
            obj.moveRobot(obj.nedModel1, obj.ik1, ...
                above_object_positions{objectIndex}, object_orientations{objectIndex}, ...
                obj.jointStateSub1, obj.nedCmdPub1);

            % Move down to the object
            obj.moveRobot(obj.nedModel1, obj.ik1, ...
                object_positions{objectIndex}, object_orientations{objectIndex}, ...
                obj.jointStateSub1, obj.nedCmdPub1);

            % Close gripper to grab
            obj.controlGripper(obj.gripperCmdPub1, obj.CmdMsg1, 2, obj.gripperStatusSub1);

            robot1PickTime = toc(robot1Pick);
            fprintf('Robot 1 Time to pick Object %d: %.2f seconds\n', objectIndex, robot1PickTime);

            % Start placing
            robot1Place = tic;

            % Move back above the object
            obj.moveRobot(obj.nedModel1, obj.ik1, ...
                above_object_positions{objectIndex}, object_orientations{objectIndex}, ...
                obj.jointStateSub1, obj.nedCmdPub1);

            % Move above work environment
            obj.moveRobot(obj.nedModel1, obj.ik1, ...
                above_work_position, work_orientation, ...
                obj.jointStateSub1, obj.nedCmdPub1);

            % Move down to place object
            obj.moveRobot(obj.nedModel1, obj.ik1, ...
                work_position, work_orientation, ...
                obj.jointStateSub1, obj.nedCmdPub1);

            % Open gripper to drop
            obj.controlGripper(obj.gripperCmdPub1, obj.CmdMsg1, 1, obj.gripperStatusSub1);

            % Move back to next "home" or staging
            obj.moveRobot(obj.nedModel1, obj.ik1, ...
                above_object_positions{objectIndex+1}, object_orientations{objectIndex+1}, ...
                obj.jointStateSub1, obj.nedCmdPub1);

            robot1PlaceTime = toc(robot1Place);
            fprintf('Robot 1 Time to place Object %d: %.2f seconds\n', objectIndex, robot1PlaceTime);

            totalTime = toc(robot1Pick);
            fprintf('Robot 1 total task Time for Object %d: %.2f seconds\n', objectIndex, totalTime);
        end

        %% --- Robot 2 Task (Pick, Classify & Place) ---
        function runRobot2Task(obj, objectIndex)
            home_position = [0.150,  0,     0.200];
            home_orientation = [0, 0.77, 0];
            work_pov_position = [ 0,     -0.310, 0.150];
            work_above_object_position = [-0.010, -0.415, 0.120];
            work_object_position =       [-0.010, -0.415, 0.095];
            cuboid_box_position   =      [-0.100,  0.300, 0.160];
            cylinder_box_position =      [ 0.100,  0.300, 0.160];
            orientation = [0, 1.540, 0];

            % Move to home & open gripper
            obj.moveRobot(obj.nedModel2, obj.ik2, ...
                home_position, home_orientation, ...
                obj.jointStateSub2, obj.nedCmdPub2);

            obj.controlGripper(obj.gripperCmdPub2, obj.CmdMsg2, 1, obj.gripperStatusSub2);  % Open

            robot2Pick = tic;

            % Move to overhead camera position
            obj.moveRobot(obj.nedModel2, obj.ik2, ...
                work_pov_position, orientation, ...
                obj.jointStateSub2, obj.nedCmdPub2);

            % Capture image & classify
            robot2Predict = tic;
            img = obj.captureImage(obj.cameraSub2);

            if isempty(obj.net)
                warning('No network loaded, skipping classification. Defaulting to "Unknown"');
                predictedLabel = "Unknown";
            else
                imgResized = imresize(img, [224 224]);
                predictedLabel = classify(obj.net, imgResized);
            end
            robot2PredictTime = toc(robot2Predict);

            fprintf('Object %d Predicted Shape: %s\n', objectIndex, char(predictedLabel));
            fprintf('Prediction Time for Object %d: %.2f seconds\n', objectIndex, robot2PredictTime);

            % Move above the object
            obj.moveRobot(obj.nedModel2, obj.ik2, ...
                work_above_object_position, orientation, ...
                obj.jointStateSub2, obj.nedCmdPub2);

            % Move down to object
            obj.moveRobot(obj.nedModel2, obj.ik2, ...
                work_object_position, orientation, ...
                obj.jointStateSub2, obj.nedCmdPub2);

            % Close gripper to grab
            obj.controlGripper(obj.gripperCmdPub2, obj.CmdMsg2, 2, obj.gripperStatusSub2);

            robot2PickTime = toc(robot2Pick);
            fprintf('Robot 2 Time to Pick Object %d: %.2f seconds\n', objectIndex, robot2PickTime);

            robot2Place = tic;

            % Move to target box depending on shape
            if strcmpi(char(predictedLabel), 'Cylinder')
                obj.moveRobot(obj.nedModel2, obj.ik2, ...
                    cylinder_box_position, orientation, ...
                    obj.jointStateSub2, obj.nedCmdPub2);
            else
                obj.moveRobot(obj.nedModel2, obj.ik2, ...
                    cuboid_box_position, orientation, ...
                    obj.jointStateSub2, obj.nedCmdPub2);
            end

            % Open gripper to drop
            obj.controlGripper(obj.gripperCmdPub2, obj.CmdMsg2, 1, obj.gripperStatusSub2);

            robot2PlaceTime = toc(robot2Place);
            fprintf('Robot 2 Time to Place Object %d: %.2f seconds\n', objectIndex, robot2PlaceTime);
            
            totalTime = toc(robot2Pick);
            fprintf('Robot 2 total task Time for Object %d: %.2f seconds\n', objectIndex, totalTime);
            
            % Return to home
            obj.moveRobot(obj.nedModel2, obj.ik2, ...
                home_position, home_orientation, ...
                obj.jointStateSub2, obj.nedCmdPub2);
        end

        %% --- Show logs for BOTH robots & do correlation analysis ---
        function showLogs(obj, baseFileName)
            if nargin < 2
                error('Please provide a base file name');
            end
            % Analyze each robot's data
            obj.analyzeRobot1Performance(baseFileName);
            obj.analyzeRobot2Performance(baseFileName);

            % Optionally plot cross-robot correlation
            obj.plotTaskCorrelation(baseFileName);
        end

        %% --- Analyze Robot1 Data (bag) ---
        function analyzeRobot1Performance(obj, baseFileName)
            bagFile = strcat(baseFileName, '_robot1.bag');
            if ~exist(bagFile, 'file')
                error('Bag file does not exist: %s', bagFile);
            end
            bag = rosbag(bagFile);
        
            % Extract available topics
            topics = bag.AvailableTopics.Properties.RowNames;
        
            % Validate and find the exact topics
            jointStateMatch = topics(contains(topics, obj.jointStateSub1.TopicName));
            jointCommandMatch = topics(contains(topics, obj.nedCmdPub1.TopicName));
            gripperCommandMatch = topics(contains(topics, obj.gripperCmdPub1.TopicName));
            gripperStatusMatch = topics(contains(topics, obj.gripperStatusSub1.TopicName));
        
            if isempty(jointStateMatch)
                error('Joint state topic not found in bag file for Robot 1.');
            end
            if isempty(jointCommandMatch)
                error('Joint command topic not found in bag file for Robot 1.');
            end
            if isempty(gripperStatusMatch)
                error('Joint command topic not found in bag file for Robot 1.');
            end
            if isempty(gripperCommandMatch)
                error('Gripper command topic not found in bag file for Robot 1.');
            end
        
            % Use the matched topic names
            jointStateTopic = jointStateMatch{1};
            jointCommandTopic = jointCommandMatch{1};
            gripperStatusTopic = gripperStatusMatch{1};
            gripperCommandTopic = gripperCommandMatch{1};
        
            % Call updated `plotJointStates` with both topics
            obj.plotJointStates(bag, jointStateTopic, jointCommandTopic);
            obj.plotGripperCommandVsStatus(bag, gripperCommandTopic, gripperStatusTopic);
        end

        %% --- Analyze Robot2 Data (bag) ---
        function analyzeRobot2Performance(obj, baseFileName)
            bagFile = strcat(baseFileName, '_robot2.bag');
            if ~exist(bagFile, 'file')
                error('Bag file does not exist: %s', bagFile);
            end
            bag = rosbag(bagFile);
        
            % Extract available topics
            topics = bag.AvailableTopics.Properties.RowNames;
        
            % Validate and find the exact topics
            jointStateMatch = topics(contains(topics, obj.jointStateSub2.TopicName));
            jointCommandMatch = topics(contains(topics, obj.nedCmdPub2.TopicName));
            gripperCommandMatch = topics(contains(topics, obj.gripperCmdPub2.TopicName));
            gripperStatusMatch = topics(contains(topics, obj.gripperStatusSub2.TopicName));
            cameraMatch = topics(contains(topics, obj.cameraSub2.TopicName));
        
            if isempty(jointStateMatch)
                error('Joint state topic not found in bag file for Robot 2.');
            end
            if isempty(jointCommandMatch)
                error('Joint command topic not found in bag file for Robot 2.');
            end
            if isempty(gripperCommandMatch)
                error('Gripper command topic not found in bag file for Robot 2.');
            end
            if isempty(gripperStatusMatch)
                error('Joint command topic not found in bag file for Robot 1.');
            end
            if isempty(cameraMatch)
                error('Camera topic not found in bag file for Robot 2.');
            end
        
            % Use the matched topic names
            jointStateTopic = jointStateMatch{1};
            jointCommandTopic = jointCommandMatch{1};
            gripperCommandTopic = gripperCommandMatch{1};
            gripperStatusTopic = gripperStatusMatch{1};
            cameraTopic = cameraMatch{1};
        
            % Call updated `plotJointStates` with both topics
            obj.plotJointStates(bag, jointStateTopic, jointCommandTopic);
            obj.plotGripperCommandVsStatus(bag, gripperCommandTopic, gripperStatusTopic);
            obj.plotCameraFeedWithGap(bag, cameraTopic, jointStateTopic, 6);
        end

        %% --- Example Correlation Plot ---
        function plotTaskCorrelation(obj, baseFileName)
            % Load the two bag files
            bagFile1 = strcat(baseFileName, '_robot1.bag');
            bagFile2 = strcat(baseFileName, '_robot2.bag');

            if ~exist(bagFile1, 'file') || ~exist(bagFile2, 'file')
                warning('One or both robot bag files do not exist. Cannot correlate.');
                return;
            end

            bag1 = rosbag(bagFile1);
            bag2 = rosbag(bagFile2);

            % Simple demonstration: compare number of messages/time
            summary1 = bag1.AvailableTopics;
            summary2 = bag2.AvailableTopics;

            disp('=== Correlation Analysis ===');
            disp('Robot1 topics:');
            disp(summary1);
            disp('Robot2 topics:');
            disp(summary2);

            % You could further read timestamps and do more advanced analysis here.
            % For demonstration, we'll just let users expand as needed.
            disp('Correlation analysis placeholder complete.');
        end

        %% --- Updated Joint States Plot with Subplots ---
        function plotJointStates(obj, bag, jointStateTopic, jointCommandTopic)
            % Validate bag file
            if isempty(bag)
                error('Invalid or empty bag file.');
            end
        
            % Select joint state messages
            jointBag = select(bag, 'Topic', jointStateTopic);
            jointMsgs = readMessages(jointBag, 'DataFormat','struct');
        
            % Select joint command messages
            commandBag = select(bag, 'Topic', jointCommandTopic);
            commandMsgs = readMessages(commandBag, 'DataFormat','struct');
        
            if isempty(jointMsgs)
                warning('No joint state messages found on topic: %s', jointStateTopic);
                return;
            end
        
            if isempty(commandMsgs)
                warning('No joint command messages found on topic: %s', jointCommandTopic);
            end
        
            % Extract joint state timestamps & positions
            jointTimes = cellfun(@(m) double(m.Header.Stamp.Sec) + ...
                                            double(m.Header.Stamp.Nsec)*1e-9, jointMsgs);
            jointPositions = cell2mat(cellfun(@(m) m.Actual.Positions(:)', ...
                                               jointMsgs, 'UniformOutput', false));
        
            % Extract joint command timestamps & positions
            commandTimes = [];
            commandPositions = [];
            if ~isempty(commandMsgs)
                commandTimes = cellfun(@(m) double(m.Header.Stamp.Sec) + ...
                                                  double(m.Header.Stamp.Nsec)*1e-9, commandMsgs);
                commandPositions = cell2mat(cellfun(@(m) m.Points(1).Positions(:)', ...
                                                      commandMsgs, 'UniformOutput', false));
            end
        
            % **Align time axis**: Use the earliest timestamp as reference
            minTime = min([jointTimes; commandTimes]);
            jointTimes = jointTimes - minTime;
            if ~isempty(commandTimes)
                commandTimes = commandTimes - minTime;
            end
        
            % Plot joint states
            figure('Name','Joint States with Commands','Position',[100 100 1000 500]);
            hold on;
            
            % Define colors based on default MATLAB line colors
            colorOrder = get(gca, 'ColorOrder');
            numJoints = size(jointPositions, 2);
        
            % Plot actual joint states
            plots = gobjects(numJoints, 1);
            for i = 1:numJoints
                plots(i) = plot(jointTimes, jointPositions(:, i), 'Color', colorOrder(mod(i-1, size(colorOrder,1))+1, :), 'LineWidth', 1.5);
            end
        
            % Overlay commanded joint angles as scatter points in the same color as the joints
            if ~isempty(commandTimes)
                for i = 1:numJoints
                    scatter(commandTimes, commandPositions(:, i), 80, '*', 'MarkerEdgeColor', colorOrder(mod(i-1, size(colorOrder,1))+1, :), 'LineWidth', 1.5);
                end
            end
        
            xlabel('Time (s)');
            ylabel('Joint Position (rad)');
            title(['Joint States with Commands: ', jointStateTopic]);
            legend([plots; scatter(NaN, NaN, '*', 'MarkerEdgeColor', 'k')], {'Joint1','Joint2','Joint3','Joint4','Joint5','Joint6','Commanded Positions'}, 'Location', 'best');
            grid on;
            hold off;
        end

        function plotGripperCommandVsStatus(obj, bag, commandTopic, statusTopic)
            % Read command messages
            cmdBag = select(bag, 'Topic', commandTopic);
            cmdMsgs = readMessages(cmdBag, 'DataFormat','struct');
            
            % Read status messages
            statusBag = select(bag, 'Topic', statusTopic);
            statusMsgs = readMessages(statusBag, 'DataFormat','struct');
        
            % Extract command times and types (1=Open, 2=Close)
            cmdTimes = cellfun(@(m) double(m.Header.Stamp.Sec) + double(m.Header.Stamp.Nsec)*1e-9, cmdMsgs);
            cmdTypes = cellfun(@(m) double(m.Goal.Cmd.CmdType), cmdMsgs); % 1=Open, 2=Close
        
            % Extract status times and SUCCEEDED (3) flags
            statusTimes = cellfun(@(m) double(m.Header.Stamp.Sec) + double(m.Header.Stamp.Nsec)*1e-9, statusMsgs);
            statusValues = cellfun(@(msg) MultiCobotController.extractStatusFromStatusList(msg.StatusList), statusMsgs);
        
            % Normalize time axis
            minTime = min([cmdTimes; statusTimes]);
            cmdTimes = cmdTimes - minTime;
            statusTimes = statusTimes - minTime;
        
            % Track gripper state based on SUCCEEDED status (3)
            gripperState = ones(size(statusTimes)) * NaN;
            lastState = 1; % Default to "Open"
            for i = 1:length(statusTimes)
                if statusValues(i) == 3 % SUCCEEDED
                    validCmdIndices = find(cmdTimes <= statusTimes(i));
                    if ~isempty(validCmdIndices)
                        [~, latestCmdIdx] = max(cmdTimes(validCmdIndices));
                        lastState = cmdTypes(validCmdIndices(latestCmdIdx));
                    else
                        warning('No command before SUCCEEDED at t=%.2f. Using lastState=%d.', statusTimes(i), lastState);
                    end
                end
                gripperState(i) = lastState;
            end
        
            % Plot
            figure('Name','Gripper Commands vs Execution (Corrected)','Position',[100 100 1000 500]);
            hold on;
            
            % Plot commands as red X markers
            scatter(cmdTimes, cmdTypes, 120, 'r', 'x', 'DisplayName','Commanded Action');
            
            % Plot actual state as blue line
            plot(statusTimes, gripperState, '-b', 'LineWidth', 2, 'DisplayName','Actual State');
            
            xlabel('Time (s)');
            ylabel('Gripper State (1=Open, 2=Close)');
            title('Gripper Command vs Execution (Corrected)');
            ylim([0.5, 2.5]);
            yticks([1 2]);
            yticklabels({'Open','Close'});
            legend show;
            grid on;
            hold off;
        end

        function plotCameraFeedWithGap(obj, bag, cameraTopic, jointStateTopic, numObjects)
            % plotCameraFeedWithGap: Captures 'numObjects' images when the robot's 
            % joint angles match `targetAngles` at least 10 seconds apart.
            %
            % This ensures if the robot reaches the same angles again after 10s, 
            % we treat it as a "new arrival" for a new object.
            %
            % INPUTS:
            %   bag             - rosbag with joint states & camera frames
            %   cameraTopic     - Camera topic (e.g. "/niryo_robot_vision/compressed_video_stream")
            %   jointStateTopic - JointTrajectoryControllerState topic
            %   targetAngles    - 1x6 array of joint angles (in rad)
            %   numObjects      - # of objects/images to capture (e.g. 6)
            %
            % LOGIC:
            %   1) Scan joint states, detect each time robot 
            %      arrives at 'targetAngles' if >=10 seconds from last arrival.
            %   2) For each arrival, find the closest camera frame.
            %   3) Display them all in a 2x3 figure.
        
            %% USER-ADJUSTABLE THRESHOLDS
            targetAngles = [-1.575, -0.520, -0.323, 0.046, -0.728, -1.608];
            angleTol   = 0.05;   % ±0.05 rad from targetAngles
            gapSeconds = 10;     % Must be 10s after previous arrival to count as "new"
        
            %% 1) Read Joint States
            jointBag = select(bag, 'Topic', jointStateTopic);
            jointMsgs = readMessages(jointBag, 'DataFormat','struct');
        
            if isempty(jointMsgs)
                warning('No joint state messages found on topic: %s', jointStateTopic);
                return;
            end
        
            % Extract times & positions
            jointTimes = cellfun(@(m) double(m.Header.Stamp.Sec) + ...
                                          double(m.Header.Stamp.Nsec)*1e-9, jointMsgs);
            jointPositions = cell2mat(cellfun(@(m) m.Actual.Positions(:)', ...
                                               jointMsgs, 'UniformOutput', false));
        
            %% 2) Detect Arrivals at targetAngles, respecting 10s gap
            arrivalTimes = [];  
            lastArrivalTime = -inf;  
        
            for i = 1:length(jointMsgs)
                currentAngles = jointPositions(i,:);
                t = jointTimes(i);
        
                % Check if "near" target angles
                if all(abs(currentAngles - targetAngles) < angleTol)
                    % Only register if 10s after the last arrival
                    if (t - lastArrivalTime) >= gapSeconds
                        arrivalTimes(end+1,1) = t; %#ok<AGROW>
                        lastArrivalTime = t;
        
                        % Stop if we've captured enough arrivals
                        if length(arrivalTimes) >= numObjects
                            break;
                        end
                    end
                end
            end
        
            % If fewer arrivals than needed
            if length(arrivalTimes) < numObjects
                warning('Found only %d arrivals but need %d for the objects.', ...
                         length(arrivalTimes), numObjects);
                numObjects = length(arrivalTimes);
            end
        
            if numObjects < 1
                warning('No valid arrival times found; skipping camera feed plotting.');
                return;
            end
        
            %% 3) Read Camera Frames
            camBag = select(bag, 'Topic', cameraTopic);
            camMsgs = readMessages(camBag, 'DataFormat','struct');
        
            if isempty(camMsgs)
                warning('No camera messages found on topic: %s', cameraTopic);
                return;
            end
        
            camTimes = cellfun(@(m) double(m.Header.Stamp.Sec) + ...
                                        double(m.Header.Stamp.Nsec)*1e-9, camMsgs);
        
            %% 4) Match Each Arrival to Closest Camera Frame
            closestCamIdx = zeros(1, numObjects);
            for i = 1:numObjects
                [~, idx] = min(abs(camTimes - arrivalTimes(i)));
                closestCamIdx(i) = idx;
            end
        
            %% 5) Display Images in a 2x3 Figure
            figure('Name','Shape Classification Images (10s Gap)','Position',[100 100 1200 600]);
            for i = 1:numObjects
                selectedMsg = camMsgs{closestCamIdx(i)};
                imgData = selectedMsg.Data;  % Compressed JPEG
        
                % Save temp file & read into MATLAB
                tempFile = [tempname, '.jpg'];
                fid = fopen(tempFile, 'w');
                fwrite(fid, imgData);
                fclose(fid);
                frame = imread(tempFile);
        
                % Show in subplot
                subplot(2, 3, i);
                imshow(frame);
                title(sprintf('Object %d at t=%.2f s', i, arrivalTimes(i)));
            end
        end

        %% --- Destructor ---
        function delete(obj)
            if obj.isRecording
                obj.stopRecording();
            end
            rosshutdown;
            disp('ROS shutdown completed. MultiCobotController deleted.');
        end
    end

    methods(Static)
        %% --- Move Robot Helper ---
        function moveRobot(ned, ik, vect, eul, jointStateSub, nedCmdPub)
            % IK + trajectory approach for a Niryo Ned

            % Joint limits
            jointLimits = [ 
                -pi,     pi;    % J1
                -pi/2,   pi/2;  % J2
                -5*pi/6, 5*pi/6;% J3
                -pi,     pi;    % J4
                -pi/2,   pi/2;  % J5
                -pi,     pi     % J6
            ];

            maxJointSpeed = 1.5;    % rad/s
            maxAcceleration = 1.2;  % rad/s^2
            accelLimits = [2.0, 2.0, 2.0, 2.0, 2.0, 2.0];

            weight = [10 10 10 1 1 1];  % IK weighting
            initialguess = ned.homeConfiguration;

            quat = quatnormalize(eul2quat(eul, 'ZYX'));
            tform = trvec2tform(vect) * quat2tform(quat);
            configSoln = ik("end_effector", tform, weight, initialguess);

            cellData = struct2cell(configSoln);
            matrixJoints = cell2mat(cellData(2, :, :));
            matrixJoints = matrixJoints(1:6);
            matrixJoints = reshape(matrixJoints, 1, []); % Force 1x6 row vector

            % Validate joint limits
            if any(matrixJoints < jointLimits(:,1)' | matrixJoints > jointLimits(:,2)')
                error('Joint limits exceeded.');
            end

            % Get current state
            try
                currentState = receive(jointStateSub, 1.5); % up to 3s wait
                currentJoints = currentState.Actual.Positions(:)';
            catch
                error('Unable to get current joint state (timeout).');
            end

            % Compute trajectory time
            jointDifferences = matrixJoints - currentJoints;
            maxJointDiff = max(abs(jointDifferences));
            finalDuration = max(1, min(maxJointDiff/maxJointSpeed, 3)); 

            % Build command
            CmdMsg = rosmessage(nedCmdPub);
            CmdMsg.JointNames = {'joint_1','joint_2','joint_3','joint_4','joint_5','joint_6'};

            CmdPoint = rosmessage('trajectory_msgs/JointTrajectoryPoint');
            CmdPoint.Positions = matrixJoints;
            CmdPoint.Velocities = jointDifferences / finalDuration;

            % Accelerations
            calculatedAccel = abs(jointDifferences) / finalDuration;  % |Δvelocity / Δtime|
            maxAllowedAccel = min(maxAcceleration, accelLimits);       % Global + per-joint limits
            CmdPoint.Accelerations = sign(jointDifferences) .* min(calculatedAccel, maxAllowedAccel);

            CmdPoint.TimeFromStart = rosduration(finalDuration);
            CmdMsg.Points = CmdPoint;
            CmdMsg.Header.Stamp = rostime("now");

            % --- Execute
            try
                send(nedCmdPub, CmdMsg);
            catch ME
                error('Send failed: %s', ME.message);
            end

            % Wait for motion to complete
            maxWaitTime = 10;
            tolerance = 0.05;
            startTime = tic;

            while toc(startTime) < maxWaitTime
%                 pause(0.05);
                try
                    currentJoints = receive(jointStateSub, 1.5).Actual.Positions(:)';
                    if norm(currentJoints - matrixJoints) < tolerance
                        % disp('Target reached');
                        return;
                    end
                catch
                    continue; 
                end
            end
            warning('Robot move timed out before reaching target.');
        end

        %% --- Control Gripper Helper ---
        function controlGripper(NedCmd, CmdMsg, action, NedState2)
            % Build tool command
            CmdGoal = rosmessage('niryo_robot_tools_commander/ToolGoal');
            CmdGoal.Cmd.ToolId = 11;           % Gripper tool ID
            CmdGoal.Cmd.CmdType = action;      % 1=open,2=close
            CmdGoal.Cmd.Speed = 500;           % 0-500 recommended
            CmdGoal.Cmd.MaxTorquePercentage = 100;
            CmdGoal.Cmd.HoldTorquePercentage  = 100;
            CmdGoal.Cmd.Activate = 1;

            % Mark the goal message
            CmdMsg.Header.Stamp = rostime("now");
            CmdMsg.GoalId.Stamp = rostime("now");

            if action == 1
                CmdMsg.GoalId.Id = "OPENGRIPPER";
            else
                CmdMsg.GoalId.Id = "CLOSEGRIPPER";
            end
            CmdMsg.Goal = CmdGoal;

            % Send the command
            send(NedCmd, CmdMsg);

            % Wait for gripper to finish
            timeout = 5;
            startTime = tic;
            while toc(startTime) < timeout
                statusMsg = receive(NedState2, 2); % wait up to 2s
                if ~isempty(statusMsg.StatusList)
                    lastStatus = statusMsg.StatusList(end).Status;
                    % Status 3 == "SUCCESS" in actionlib
                    if lastStatus == 3
                        return;  % Done
                    end
                end
            end
            warning('Gripper action timed out or incomplete.');
        end

        %% --- Capture Image from Robot 2 Camera ---
        function img = captureImage(cameraSub)
            % Wait for up to 10s for a camera image
            compressedImage = receive(cameraSub, 10);
            if isempty(compressedImage)
                warning('No image received from camera within 10s');
                img = [];
                return;
            end

            % Write to temp .jpg
            imgData = compressedImage.Data;
            tempFile = [tempname, '.jpg'];
            fid = fopen(tempFile, 'w');
            fwrite(fid, imgData);
            fclose(fid);

            % Read as standard MATLAB image
            img = imread(tempFile);
            % Optional: imshow(img);  % For debug
        end

        %% --- Calibrate Robot (Optional) ---
        function calibrateRobot(node)
            % Request new calibration
            client = ros.ServiceClient(node, '/niryo_robot/joints_interface/request_new_calibration');
            req = rosmessage(client);
            resp = call(client, req);

            if resp.Status == 1
                disp(['Calibration request success for ', node.Name]);
            else
                disp(['Calibration request failed for ', node.Name]);
                return;
            end
            disp(['Message: ', resp.Message]);

            % Calibrate motors
            client = ros.ServiceClient(node, '/niryo_robot/joints_interface/calibrate_motors');
            req = rosmessage(client);
            req.Value = 1;   % Start calibration
            resp = call(client, req);
            disp(['Calibration Response: ', resp.Message]);
        end

        function val = extractStatusFromStatusList(statusList)
            % Handle empty or last status
            if isempty(statusList)
                val = NaN;  % or 0 => meaning "no status"
            else
                val = double(statusList(end).Status);
            end
        end
    end
end