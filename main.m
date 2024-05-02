addpath("Neato\")
load("final_lidar_data.mat");

angle_rotated = pi/2;
position_moved = [0 0];
clf;

angular_speed = pi/8;
linear_speed = 0.07;
wheel_angular_speed = (angular_speed * 0.245)/2;

clf;

mainframe_pos = [mainframe_pos1 mainframe_pos2];
%scatter(mainframe_pos1(1, :), mainframe_pos1(2, :)); hold on;
%scatter(mainframe_pos(1, :), mainframe_pos(2, :));

[x,y]=meshgrid(-2:0.01:2,-2:0.01:2);
v = 0;
for i = 1:size(mainframe_pos,2)
    a = mainframe_pos(1,i);
    b = mainframe_pos(2,i);
    v = v + 0.008./((x-a).^2 + (y-b).^2);
end
v = v + 8 .*  log(sqrt((x+1.1).^2 + (y+0).^2));
contour(x, y, v, linspace(-20, 100, 20)); hold on;
xlim([-2 1])
ylim([-0.6 1.7])
xlabel("X (m)")
ylabel("Y (m)")
%surf(x, y, v);
%clim([-10 25])
%zlim([-10 25]);

[fx, fy] = gradient(v);
fx = -0.04 .* fx./sqrt(fx.^2 + fy.^2);
fy = -0.04 .* fy./sqrt(fx.^2 + fy.^2);
%quiver(x, y, fx, fy); hold on;

%neatov2.connect('192.168.16.115');
%neatov2.testConnection();

for i=1:40
    dist_x = round(position_moved(1), 2);
    dist_y = round(position_moved(2), 2);

    x_index = find(abs(x(1, :) - dist_x) < 0.00001);
    y_index = find(abs(y(:, 1) - dist_y) < 0.00001);

    gradient_x = fx(y_index, x_index);
    gradient_y = fy(y_index, x_index);
    
    gradient_angle = atan2(gradient_y, gradient_x);
    if gradient_angle < 0
        gradient_angle = 2 * pi + gradient_angle;
    end
    change_angle = gradient_angle - angle_rotated;
    angle_rotated = gradient_angle;
    time_to_rotate = change_angle/angular_speed;
    rotate_direction = time_to_rotate/abs(time_to_rotate);
    %neatov2.driveFor(abs(time_to_rotate), rotate_direction * -wheel_angular_speed,rotate_direction * wheel_angular_speed, false);
    
    %neatov2.plotSim();
    quiver(position_moved(1), position_moved(2), gradient_x, gradient_y); hold on;
    position_moved = position_moved + [gradient_x gradient_y];
    time_to_drive = norm([gradient_x gradient_y])/linear_speed;
    %neatov2.driveFor(time_to_drive, linear_speed, linear_speed, false);
    
    %neatov2.plotSim();
end