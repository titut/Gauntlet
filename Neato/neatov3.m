classdef neatov3 < handle
    %neatov2 Manages a connection from MATLAB to the Neato robot
    %   neatov2 allows you to establish a connection between your computer
    %   and one of the Neato robots.  In order for the system to function
    %   properly, your computer should be on the OLIN-ROBOTICS network.
    %
    %   The class provides basic functionalities for reading sensor data,
    %   sending motor commands, connecting to the robot, and disconnecting
    %   from the robot.

    properties(Access=private)
        lowBatteryTimer
        sock
        port_num
        isSimulated
        timeStamp
        robotState
        xvals
        yvals
        zvals
    end

    methods(Static)
        function obj = connect(varargin)
            %CONNECT
            %   Create a connection to the Neato.  If no arguments are
            %   supplied the simulator will be launched.  If an IP address
            %   is specified (as the first argument), a connection to the
            %   physical robot will be created.  An optional second
            %   argument can be specified to change the UDP port that the
            %   sensor data is sent on.
            obj = neatov3.getInstance(true, varargin{:});
        end

        function disconnect()
            %DISCONNECT
            %   Disconnect from the Neato
            obj = neatov3.getInstance(false);
            delete(obj);
        end

        function s = receive()
            %RECEIVE
            %    Fetch the most recent sensor data from the robot.  If the
            %    robot has low batteries, this will be flagged here.
            obj = neatov3.getInstance(false);
            s = receiveImpl(obj);
        end

        function setPositionAndOrientation(x, y, theta)
            %SETPOSITIONANDORIENTATION
            %   The robot's position and orientation will be set according
            %   to the inputs.
            obj = neatov3.getInstance(false);
            setPositionAndOrientationImpl(obj, x, y, theta);
        end

        function setFlatlandContours(xvals, yvals, zvals)
            %SETFLATLANDCONTOURS
            %  Contours of this surface will be drawn on top of the
            %  simulator visualization.
            obj = neatov3.getInstance(false);
            setFlatlandContoursImpl(obj, xvals, yvals, zvals);
        end

        function testConnection()
            %TESTCONNECTION
            %  Send data to the Neato to see if the connection is still
            %  alive.  This sometimes has to be called more than once to
            %  detect a dropped connection.
            obj = neatov3.getInstance(false);
            testConnectionImpl(obj);
        end

        function driveFor(seconds, vL, vR, drawPlot)
            %DRIVEFOR
            %  Drive for the the specified number of seconds with the
            %  indicated wheel velocities. If drawPlot is
            %  true, draw the simulator plot.
            obj = neatov3.getInstance(false);
            if nargin < 4
                drawPlot = false;
            end
            driveForImpl(obj, seconds, vL, vR, drawPlot);
        end

        function plotSim()
            %PLOTSIM
            %    Draw a representation of the simulated robot.
            obj = neatov3.getInstance(false);
            plotSimImpl(obj);
        end


        function setVelocities(vl, vr)
            %SETVELOCITIES
            %   The robot will move according to the specified left and
            %   right wheel velocities (in meters per second)
            %   If no velocities are sent to the robot, it will stop after
            %   half a second or so.
            obj = neatov3.getInstance(false);
            setVelocitiesImpl(obj, vl, vr);
        end

        function sendKeepAlive()
            %SENDKEEPALIVE
            %    Send a message to the physical robot to prevent the
            %    connection from timing out.
            obj = neatov3.getInstance(false);
            sendKeepAliveImpl(obj);
        end
    end

    methods

    end

    methods (Access=private)
        function obj = neatov3(ip,varargin)
            %neatov2
            %   Create a connection to the Neato.  If no arguments are
            %   supplied the simulator will be launched.  If an IP address
            %   is specified (as the first argument), a connection to the
            %   physical robot will be created.  An optional second
            %   argument can be specified to change the UDP port that the
            %   sensor data is sent on.
            if nargin>1
                obj.port_num=varargin{1};
            else
                obj.port_num=7921;
            end
            obj.lowBatteryTimer = tic;
            if nargin > 0
                obj.sock = tcpclient(ip, 7777, "ConnectTimeout", 5);
                pause(1);
                txt2=sprintf('protocolpreference True %d False', obj.port_num);
                writeline(obj.sock, txt2);
                pause(1);
                writeline(obj.sock, 'testmode on');
                pause(1);
                writeline(obj.sock, 'setldsrotation on');
                pause(2);
                obj.isSimulated = false;
                disp('Testing connection.');
                receiveImpl(obj);
                disp('Connection successful.');
            else
                obj.isSimulated = true;
                obj.timeStamp = [];
                obj.robotState = [0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0];
            end
        end

        function setMotors(obj, l, r, s)
            if isnan(l) || isnan(r) || isnan(s)
                warning('wheel velocities contain NaN values')
                return
            end
            if isinf(l) || isinf(r) || isinf(s)
                warning('wheel velocities contain Inf values')
                return
            end
            if abs(l/1000) > 0.3 || abs(r/1000.0) > 0.3
                warning('trying to set wheel speeds greater than 0.3 m/s')
                return
            end
            if obj.isSimulated
                % update speeds here
                obj.robotState(4) = l/1000.0;
                obj.robotState(5) = r/1000.0;
            else
                if l == 0 && r == 0 && s == 0
                    writeline(obj.sock, 'setmotor 1 1 1');
                end
                writeline(obj.sock, sprintf('setmotor %d %d %d', l, r, s));
            end
        end

        function newState = forwardKinematics(obj, deltaT)
            BASE_WIDTH = 248;
            % https://www.cs.columbia.edu/~allen/F17/NOTES/icckinematics.pdf
            l = BASE_WIDTH/1000.0;
            vF = (obj.robotState(4) + obj.robotState(5))/2.0;
            omega = (obj.robotState(5) - obj.robotState(4))/l;
            newState = obj.robotState;
            R = (obj.robotState(4) + obj.robotState(5))/(obj.robotState(5) - obj.robotState(4))*l/2.0;
            if isfinite(R)
                ICC = [obj.robotState(1) - R*sin(obj.robotState(3)) obj.robotState(2) + R*cos(obj.robotState(3))];
                newState(1:3) = [cos(omega*deltaT) -sin(omega*deltaT) 0;...
                                 sin(omega*deltaT) cos(omega*deltaT) 0;...
                                 0 0 1]*[obj.robotState(1) - ICC(1);...
                                         obj.robotState(2) - ICC(2);...
                                         obj.robotState(3)] + [ICC(1);...
                                                      ICC(2);...
                                                      omega*deltaT];
    
            else
                newState(1:2) = [cos(obj.robotState(3));...
                                 sin(obj.robotState(3))]*deltaT*vF + [obj.robotState(1);...
                                                            obj.robotState(2)];
            end
            newState(6) = newState(6) + obj.robotState(4)*deltaT;
            newState(7) = newState(7) + obj.robotState(5)*deltaT;
            obj.robotState = newState;
        end

        function P = drawRobotBody(obj)
            robotWidth = 0.35;   % just for visualization purposes (didn't actually measure)
            P = polyshape([obj.robotState(1)-robotWidth/2, obj.robotState(2) - robotWidth/2;...
                   obj.robotState(1)-robotWidth/2, obj.robotState(2) + robotWidth/2;...
                   obj.robotState(1)+robotWidth/2, obj.robotState(2) + robotWidth/2;...
                   obj.robotState(1)+robotWidth/2, obj.robotState(2) - robotWidth/2]);
            P = rotate(P, rad2deg(obj.robotState(3)), obj.robotState(1:2)');
        end

        function [neatoSensors] = receiveImpl(obj)
            %RECEIVE
            %    Fetch the most recent sensor data from the robot.  If the
            %    robot has low batteries, this will be flagged here.
            if ~obj.isSimulated
                neatoSensors = NeatoSensors();
                packet = judp('receive', obj.port_num, 1600);
                accelAsDoubles = double(typecast(packet(1:24), 'single')');
                motorsAsDoubles = typecast(packet(25:40),'double')'/1000.0;
                digitalAsDoubles = typecast(packet(41:72),'double')';
                rangesAsDoubles = double(typecast(packet(73:end-4),'uint16'))'/1000.0;
                voltageAsDouble = double(typecast(packet(end-3:end-2), 'uint16'))/1000.0;
                fuelPercentAsDouble = double(typecast(packet(end-1:end), 'uint16'));
                
                % copy the values from the UDP packet into the shared struct
                neatoSensors.bumpers = digitalAsDoubles;
                neatoSensors.ranges = rangesAsDoubles;
                neatoSensors.thetasInRadians = deg2rad(0:359);
                neatoSensors.accels = accelAsDoubles;
                neatoSensors.encoders = motorsAsDoubles;
                neatoSensors.batteryVoltage = voltageAsDouble;
                neatoSensors.fuelPercent = fuelPercentAsDouble;
    
                if fuelPercentAsDouble < 30
                    timeSinceLastBatteryWarning = toc(obj.lowBatteryTimer);
                    if timeSinceLastBatteryWarning > 2.0
                        obj.lowBatteryTimer = tic;
                        warning('Neato has low battery');
                    end
                end
            else
                if ~isempty(obj.timeStamp)
                    deltaT = toc(obj.timeStamp);
                    forwardKinematics(obj, deltaT);
                end
                obj.timeStamp = tic;
                % generate these
                accelAsDoubles = [0.0, 0.0, 0.0, 0.0, 1.0, 1.0];    % only model acceleration due to gravity
                digitalAsDoubles = zeros(1, 4);         % not yet supported
                rangesAsDoubles = zeros(1, 360);        % not yet supported
    
                % copy the values from the UDP packet into the shared struct
                neatoSensors.bumpers = digitalAsDoubles;
                neatoSensors.ranges = rangesAsDoubles;
                neatoSensors.thetasInRadians = deg2rad(0:359);
                neatoSensors.accels = accelAsDoubles;
                neatoSensors.encoders = [obj.robotState(6) obj.robotState(7)];
            end
        end


        function setPositionAndOrientationImpl(obj, x, y, theta)
            %SETPOSITIONANDORIENTATION
            %   The robot's position and orientation will be set according
            %   to the inputs.
            if ~obj.isSimulated
                error("This can only be used on a simulated robot")
            end
            obj.robotState(1) = x;
            obj.robotState(2) = y;
            obj.robotState(3) = theta;
        end


        function setFlatlandContoursImpl(obj, xvals, yvals, zvals)
            %SETFLATLANDCONTOURS
            %  Contours of this surface will be drawn on top of the
            %  simulator visualization.
            obj.xvals = xvals;
            obj.yvals = yvals;
            obj.zvals = zvals;
        end


        function testConnectionImpl(obj)
            if ~obj.isSimulated
                try
                    sendKeepAliveImpl(obj);
                    pause(1.0);
                    sendKeepAliveImpl(obj);
                    receiveImpl(obj);
                    disp('The connection appears to be working.')
                    disp('waiting for the LIDAR to return valid date')
                    for i=1:10
                        s = neatov3.receive();
                        if sum(s.ranges) > 0
                            break
                        end
                    end
                catch ex
                    error("Neato connection has timed out, please reconnect.");
                end
            end
        end

        function driveForImpl(obj, seconds, vL, vR, drawPlot)
            %DRIVEFOR
            %  Drive for the the specified number of seconds with the
            %  indicated wheel velocities. If drawPlot is
            %  true, draw the simulator plot.
            if nargin < 5
                drawPlot = false;
            end
            dt = 0.01;
            start = tic;
            while true
                setVelocitiesImpl(obj, vL, vR);
                receiveImpl(obj);
                if drawPlot
                    plotSimImpl(obj);
                end
                pause(dt);
                elapsed = toc(start);
                if elapsed > seconds
                    break;
                end
            end
            setVelocitiesImpl(obj, 0.0, 0.0);
        end

        function plotSimImpl(obj)
            %PLOTSIM
            %    Draw a representation of the simulated robot.
            if ~obj.isSimulated
                error("This can only be used on simulated Neato");
            end
            % run forward kinematics if possible
            if ~isempty(obj.timeStamp)
                deltaT = toc(obj.timeStamp);
                forwardKinematics(obj, deltaT);
            end
            obj.timeStamp = tic;
            set(gca,'Nextplot','ReplaceChildren');
            P = drawRobotBody(obj);
            plot(P);
            hold on;
            quiver(obj.robotState(1), obj.robotState(2), cos(obj.robotState(3)), sin(obj.robotState(3)), 0.5, 'color', 'r', 'maxheadsize', 2, 'linewidth', 1);

            if ~isempty(obj.xvals)
                contour(obj.xvals, obj.yvals, obj.zvals, 20);
                colorbar;
            else
                boundary=5;
                xlim([-boundary boundary]);
                ylim([-boundary boundary]);
            end
            axis equal;
            grid on;
            drawnow;
        end

        function setVelocitiesImpl(obj, vl, vr)
            %SETVELOCITIES
            %   The robot will move according to the specified left and
            %   right wheel velocities (in meters per second)
            %   If no velocities are sent to the robot, it will stop after
            %   half a second or so.
            
            % if we are using the simulated robot, we should first update
            % the kinematics before setting these
            if obj.isSimulated && ~isempty(obj.timeStamp)
                deltaT = toc(obj.timeStamp);
                forwardKinematics(obj, deltaT);
            end
            obj.timeStamp = tic;

            setMotors(obj, ...
                      round(1000*vl), ...
                      round(1000*vr), ...
                      round(1000*max(abs([vl, vr]))));
        end

        function sendKeepAliveImpl(obj)
            %SENDKEEPALIVE
            %    Send a message to the physical robot to prevent the
            %    connection from timing out.
            if obj.isSimulated
                error("This can only be used on a real Neato (not the simulator")
            end
            writeline(obj.sock, 'keepalive');
        end
    end
   
    methods(Static, Access=private)
        function obj = getInstance(createInstance, varargin)
           persistent theInstance;
           persistent cleanup;
           if ~isempty(theInstance) && createInstance
               disp('Deleting previous Neato connection.');
               delete(theInstance);
           end
           if createInstance
               disp('Connecting to the Neato.');
               theInstance = neatov3(varargin{:});
               cleanup = onCleanup(@()delete(theInstance));
           else
               if isempty(theInstance)
                   error('Neato is not connected');
               end
           end
           obj = theInstance;
        end
    end

end