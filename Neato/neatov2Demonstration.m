function neatov2Demonstration()
    % connect to the simulator (pass the IP address to connect to the
    % actual Neato robot, e.g., neatov2.connect('192.168.16.55');
    neatov2.connect();

    % set the left wheel velocity to 0.1 m/s and the right to 0.2 m/s for a
    % total of 3.0 seconds.  The last argument controls whether to plot the
    % simulator visualization.  In this case we plot the simulator and
    % create a figure for it to be displayed within

    f = figure;
    neatov2.driveFor(3.0 , 0.1, 0.2, true);

    % the receive command will fetch the latest sensor data from the robot.
    % For the simulator we only have the encoders, so let's look at those.
    s = neatov2.receive();
    fprintf("left travel %0.3f meters, right travel %0.3f meters\n", ...
        s.encoders(1), ...
        s.encoders(2));

    % for the simulator only, you can set the position and orientation
    neatov2.setPositionAndOrientation(0.5, -0.4, -pi/2);

    % for the simulator only, you can trigger a visualization of the
    % position and orientation of the Neato
    neatov2.plotSim();

    % instead of using the driveFor command, you can also write your own
    % write your own velocities.  The following code drives until the left
    % wheel and right wheel have eached move 1.0 meter.  When looping like
    % this, there are a few things to keep in mind.  First, you need to
    % call the receive command to both get the sensor data and give the
    % simulator a chance to update.  Second, if you are connected to the
    % real robot, you need to send velocity commands at least once every
    % half a second or so or the Neato will stop.
    initialEncoders = neatov2.receive().encoders;
    currentEncoders = initialEncoders;

    while currentEncoders(1) - initialEncoders(1) < 1.0
        neatov2.plotSim();
        neatov2.setVelocities(0.075, 0.075);
        currentEncoders = neatov2.receive().encoders;
    end
    fprintf("left wheel traveled %0.3f meters\n", ...
        currentEncoders(1) - initialEncoders(1));
end