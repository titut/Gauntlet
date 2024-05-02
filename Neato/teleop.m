function [neatoSensors, neatoVelocities] = teleop()
% This script provides a visualization and teleop interface to the Neato
% robots.  The script assumes you have already connected to the Neato
% before running this.
%
% Running this script will startup a figure window that will show the 
% heading of the robot along with the laser scan data in the odometry
% coordinate system of the robot.
%
% To control the robot with the keyboard, you must have the focus on the
% visualizer window (i.e. click on the window).  The key mappings are:
%     i : forward
%     k : stop
%     j : left
%     l : right
%     , : backward
%     u : forward while turning left
%     o : forward while turning right
%     m : backward while turning left
%     . : backward while turning right
%
% Additionally there are sliders that control the both the forward and
% angular speed of the robot.
%
% To stop execution of the program, simply close the figure window.
    function setLinearVelocity(hObject, eventdata, handles)
        % callback function for the linear velocity slider
        v = get(hObject, 'Value');
    end

    function myCloseRequest(src,callbackdata)
        % Close request function 
        % to display a question dialog box
        % get rid of subscriptions to avoid race conditions
        if exist('T','var')
            stop(T);
        end
        delete(gcf);
    end

    function setAngularVelocity(hObject, eventdata, handles)
        % callback function for the angular velocity slider
        w = get(hObject, 'Value');
    end

    function keyPressedFunction(fig_obj, eventDat)
        % Convert a key pressed event into a twist message and publish it
        ck = get(fig_obj, 'CurrentKey');
        switch ck
            case 'i'
                linear = v;
                angular = 0;
            case 'u'
                linear = v;
                angular = w;
            case 'j'
                linear = 0;
                angular = w;
            case 'm'
                linear = -v;
                angular = -w;
            case 'comma'
                linear = -v;
                angular = 0;
            case 'period'
                linear = -v;
                angular = w;
            case 'l'
                linear = 0;
                angular = -w;
            case 'o'
                linear = v;
                angular = -w;
            otherwise
                linear = 0;
                angular = 0;
        end
        cmdVel(linear, angular);
    end
    v = 0.3;
    w = 0.8;
	f = figure('CloseRequestFcn',@myCloseRequest);
    neatoVelocities = NeatoVelocities();
    set(gca,'position',[.05,.20,.9,.7]);

    sld = uicontrol('Style', 'slider',...
        'Min',0,'Max',0.3,'Value',0.3,...
        'Position', [200 20 120 20],...
        'Callback', @setLinearVelocity);
    % Add a text uicontrol to label the slider.
    txt = uicontrol('Style','text',...
        'Position',[200 45 120 20],...
        'String','Linear Velocity Scale');
    sld = uicontrol('Style', 'slider',...
        'Min',0,'Max',0.6/.248,'Value',w,...
        'Position', [20 20 120 20],...
        'Callback', @setAngularVelocity);
    % Add a text uicontrol to label the slider.
    txt = uicontrol('Style','text',...
        'Position',[20 45 120 20],...
        'String','Angular Velocity Scale');

    set(f,'WindowKeyPressFcn', @keyPressedFunction);
    BASE_WIDTH = 248;    % millimeters
    MAX_SPEED = 300;     % millimeters/second
    t = tic;
    T = timer('Period',0.2,... %period
        'ExecutionMode','fixedRate',... %{singleShot,fixedRate,fixedSpacing,fixedDelay}
        'BusyMode','drop',... %{drop, error, queue}
        'TasksToExecute',inf,...          
        'StartDelay',1,...
        'TimerFcn',@(src,evt)socketLoop(),...
        'StartFcn',[],...
        'StopFcn',[],...
        'ErrorFcn',[]);
    start(T);

    function socketLoop()
        elapsedSinceKeepAlive = toc(t);
        if elapsedSinceKeepAlive > 10.0
            t = tic;
            neatov2.sendKeepAlive();
        end
        try
            if length(neatoVelocities.lrWheelVelocitiesInMetersPerSecond) == 2
                neatov2.setVelocities(neatoVelocities.lrWheelVelocitiesInMetersPerSecond(1), ...
                    neatoVelocities.lrWheelVelocitiesInMetersPerSecond(2))
                if sum(abs(neatoVelocities.lrWheelVelocitiesInMetersPerSecond)) == 0
                    % don't continue to send stop command in case we have
                    % a program running
                    neatoVelocities.lrWheelVelocitiesInMetersPerSecond = [];
                end
            end
            neatoSensors = neatov2.receive();
            
            prev = gcf;
            set(0,'CurrentFigure',f);
            set(gca,'Nextplot','ReplaceChildren');
            polarplot(neatoSensors.thetasInRadians, neatoSensors.ranges, 'b.');
            rlim([0 5]);
            hold off;
            if isvalid(prev)
                set(0, 'CurrentFigure', prev);
            end
        catch ex
            %ex
        end
        drawnow;
    end
    function cmdVel(linear, angular)
        x = linear * 1000;
        th = angular * (BASE_WIDTH/2);
        k = max(abs(x-th),abs(x+th));
        if k > MAX_SPEED
            x = x*MAX_SPEED/k; th = th*MAX_SPEED/k;
        end
        cmd_vel = [ round(x-th) , round(x+th) ];
        neatoVelocities.lrWheelVelocitiesInMetersPerSecond = cmd_vel/1000.0;
    end

end
