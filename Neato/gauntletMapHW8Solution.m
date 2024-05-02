function gauntletMapHW8Solution()
    load('hw8sampledata','angles','positions','scans');
    allPlots = figure;
    title('Superimposed Plots');
    for i=1:size(positions,1)
        th = deg2rad(angles(i));
        RobotToGlobal = [cos(th) -sin(th) positions(i,1);
                         sin(th) cos(th) positions(i,2);
                         0 0 1];
        % the scanner is located about 8cm behind the center of rotation of
        % the robot
        ScannerToRobot = [1 0 -0.082;
                          0 1 0;
                          0 0 1];

        ScannerToGlobal = RobotToGlobal * ScannerToRobot;
        ranges = scans{i}.ranges;
        thetasInRadians = scans{i}.thetasInRadians;
        cart = [cos(thetasInRadians);
                sin(thetasInRadians)].*[ranges; ranges];
        cartHomogeneous = [cart; ones(1, size(cart,2))];
        cleaned = cartHomogeneous(:, ranges~=0);
        globalCleaned = ScannerToGlobal*cleaned;
        figure;
        title(['x=',num2str(positions(i,1)), ...
                ' y=', num2str(positions(i,2)),...
                ' angles=', num2str(angles(i))]);
        scatter(globalCleaned(1,:), globalCleaned(2,:), 'b.');
        axis("equal");
        figure(allPlots);
        hold on;
        scatter(globalCleaned(1,:), globalCleaned(2,:), 'b.');
        axis("equal");
    end

end