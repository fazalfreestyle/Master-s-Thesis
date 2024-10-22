function img = captureImage(cameraSub)
    % Wait for and receive the image message
    compressedImage = receive(cameraSub, 10);  % Timeout after 10 seconds
    
    % Extract the compressed image data (in JPEG format)
    imgData = compressedImage.Data;  % Image data is of type uint8
    
    % Create a temporary file to store the compressed image
    tempFile = tempname;  % Create a temporary file
    fid = fopen([tempFile, '.jpg'], 'w');  % Open the temp file to write the image
    fwrite(fid, imgData);  % Write the image data to file
    fclose(fid);  % Close the file
    
    % Read the image back using imread
    img = imread([tempFile, '.jpg']);
    
    % Optionally display the captured image for debugging
    imshow(img);
end
