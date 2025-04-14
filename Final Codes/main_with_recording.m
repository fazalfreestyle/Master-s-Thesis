function main_with_recording()
    % Clean up previous instances
    if exist('controller', 'var') && isvalid(controller)
        delete(controller);
        clear controller;
        disp('Old instance deleted.');
    end
    
    % Initialize controller with robot parameters
    controller = MultiCobotController("192.168.47.190", "192.168.47.152", ...
        "C:\Users\fazal\Downloads\ned2\niryo_robot_description\urdf\ned2\niryo_ned2.urdf", ...   % Robot 1 URDF
        "C:\Users\fazal\Downloads\ned2\niryo_robot_description\urdf\ned2\niryo_ned2.urdf", ...   % Robot 2 URDF
        "C:\Users\fazal\OneDrive\Documents\University of Michigan\Thesis\Matlab Codes\Dataset\Trained_Model\shape_classification_model_resnet50.mat");  % Net path
    
    try
        % Start recording with callback-based logging
        bagFileName = "my_robot_log";
        controller.startRecording(bagFileName);
        
        % Execute robot tasks
        for i = 1:6
            controller.runRobot1Task(i);
            controller.runRobot2Task(i);
        end
        
        % Stop recording and process results
        controller.stopRecording();
        pause(1);  % Brief pause to ensure final messages are written
        
        % Analyze and display logs
        controller.showLogs(bagFileName);
        
    catch ME
        % Handle exceptions and ensure cleanup
        controller.stopRecording();
        rethrow(ME);
    end
    
    % Cleanup
    delete(controller);
    disp('Execution completed successfully.');
end