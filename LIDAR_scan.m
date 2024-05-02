addpath("Neato\")
neatov2.connect('192.168.16.109');
neatov2.testConnection();

angle_rotated = pi/2;
position_moved = [-1.1 0.37];

%while true
sensors = neatov2.receive();
r = sensors.ranges;
r_iz_not_zero = r ~= 0;
r = r(r_iz_not_zero);
theta = sensors.thetasInRadians(r_iz_not_zero);
d = 0.09;

x = (r .* cos(theta) - d);
y = (r .* sin(theta));
one = ones(size(x));

lidar_pos = [x; y; one];

mainframe_pos2 = translatePos(position_moved(1), position_moved(2)) * rotAngle(angle_rotated) * lidar_pos;
clf;
scatter(0, 0, 300, "r"); hold on;
scatter(mainframe_pos(1, :), mainframe_pos(2, :), 10, "b"); hold off;
axis equal;