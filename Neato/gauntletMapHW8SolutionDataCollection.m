function [positions, angles, scans] = gauntletMapHW8SolutionDataCollection()
    % connect to the Neato
    neatov2.connect('192.168.17.207');
    neatov2.testConnection();
    % these are the angles and positions we will test.  There will be a
    % pause to alert the user to reposition the Neato between each of these
    % the angles are defined in degrees using counterclockwise rotations
    % relative to the x-axis
    angles = [0.0; 45.0; 90; 180];
    positions = [0.0, 0.0;
                 0.0, 0.0;
                 0.5, 1.0;
                 0.75, 1.25];
                 
    scans = cell(4,1);

    for i = 1:size(positions,1)
        disp(['Move the Neato to position x=', ...
               num2str(positions(i,1)), ...
               ' y=', ...
               num2str(positions(i,2)), ...
               ' at an angle of ', ...
               num2str(angles(i)), ...
               ' degrees counterclockwise from the x-axis']);
        pause;
        scans{i} = neatov2.receive();
    end
    save('hw8sampledata','angles','positions','scans');
end