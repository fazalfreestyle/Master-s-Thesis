function waitForCompletion(sub, completionMessage)
    disp('Waiting for task completion message...');
    while true
        msg = receive(sub, 30);  % Increased Timeout to 30 seconds
        if strcmp(msg.Data, completionMessage)
            disp(['Received task completion message: ', msg.Data]);
            break;
        end
        pause(1);
    end
end
