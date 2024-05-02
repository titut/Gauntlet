function flatlandStarter(use_sim)
%FLATLANDSTARTER
%  Load the
if nargin < 1
    use_sim = true;
end

% we put these for your convenience.  These would track position and
% orientation of your Neato.
position = [1; 0];
heading = [1; 0];

if use_sim
    neatov2.connect();
else
    % make sure to fill this out
    neatov2.connect('YOUR_NEATO_IP_HERE');
end
[xvals, yvals] = meshgrid(linspace(-1.25,1.25,100), linspace(-0.5, 0.5, 100));
zvals = 8*exp(-1.5*(xvals + yvals - 0.75).^2 - 0.5*(xvals - yvals - 0.75).^2) + 6*exp(-(xvals+0.75).^2-(yvals+0.25).^2);

if use_sim
    % For simulated Neato only we can set the position
    neatov2.setPositionAndOrientation(position(1), position(2), atan2(heading(2), heading(1)));
    % Plot the contours for the simulator
    neatov2.setFlatlandContours(xvals, yvals, zvals);
    
    f = figure;
    neatov2.plotSim();
end
end