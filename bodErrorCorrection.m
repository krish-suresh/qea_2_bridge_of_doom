Kp_heading = 0.6;
Ki_heading = 0;
Kd_heading = 0.7;
Kp_axial = 0.7;
Ki_axial = 0;
Kd_axial = 0.1;

display_vel_graph = false;
display_pos_graph = true;
display_error_graph = true;
physical_neato = true;
R_fun = @(theta) ([cos(theta) -sin(theta); sin(theta) cos(theta)]);
syms u b;

% this is the equation of the bridge

R = [0.396*cos(2.65*(u+1.4));
    -0.99*sin(u+1.4);
    0];

assume(b, 'real');
assume(b, 'positive');
assume(u, 'real');

% tangent vector
T = diff(R);

% normalized tangent vector
That = T/norm(T);
N = diff(That);

Nhat = N/norm(N);
% angular velocity vector
B = cross(That, N);
omega = B(3);
speed = norm(T);
d = 0.235;

total_dist = vpa(int(norm(T),u ,[0, 3.1]))

acc = 0.05;
max_velocity = 0.1;
t_1 = max_velocity / acc;

t_end = total_dist/max_velocity + t_1
%Time to begin deceleration 
t_2 = t_end - t_1;


%Velocity piecewise function
motion_profile = piecewise(b < t_1, (b*acc), t_1 < b < t_2, max_velocity, b > t_2, max_velocity-(b-t_2)*acc);
distance_traveled = int(motion_profile, b);
pub = rospublisher('raw_vel');
enc = rossubscriber('/encoders');
% clock = rossubscriber('/clock');
% imu = rossubscriber('/imu');
% accel = rossubscriber("/accel");
%     stop the robot if it's going right now
stopMsg = rosmessage(pub);
stopMsg.Data = [0 0];
send(pub, stopMsg);

bridgeStart = double(subs(R,u,0));
startingThat = double(subs(That,u,0));
if ~physical_neato
placeNeato(bridgeStart(1),  bridgeStart(2), startingThat(1), startingThat(2),0.5);
end
% wait a bit for robot to fall onto the bridge
pause(2);
d = 0.235;
t_last = 0;
[msg2,status,statustext] = receive(enc,10);
enc_last = msg2.Data;
% [imuMsg,status,statustext] = receive(imu,10);
% heading = quat2eul([imuMsg.Orientation.X, imuMsg.Orientation.Y, imuMsg.Orientation.Z, imuMsg.Orientation.W]);


pose = [bridgeStart(1),  bridgeStart(2),atan2(startingThat(2), startingThat(1))];
poses = pose;
if display_pos_graph
    figure(2);
    clf;
    fplot(R(1),R(2),[0 double(total_dist)]);  hold on;
    xlabel("x (meters)")
    ylabel("y (meters)")
    axis([-4, 4, -4, 4]);
    axis padded;
    axis equal;
end
if display_vel_graph
    figure(1);
    clf;
    xlabel("time (seconds)");
    axis([0, double(t_end), 0, double(total_dist)]);
end

if display_error_graph
    figure(3);
    clf;
    xlabel("time (seconds)");
end

cur_dist = 0;
cur_vel = 0;
t_delta = 0;
t = 0;
pose_error_rot_last = [0,0,0];
rostic;

while t < t_end
    t = rostoc();
    t_delta = t-t_last;
% Drive robot
    target_position = subs(R, u,cur_dist);
    target_heading_vec = subs(That, u, cur_dist);
    target_heading = atan2(target_heading_vec(2), target_heading_vec(1));
    target_pose = [double(target_position(1)), double(target_position(2)), double(target_heading)];
%     pose
%     target_pose;
    pose_error = target_pose - pose;
    pose_error_rot = [(R_fun(pi/2 - pose(3))*pose_error(1,1:2)')', pose_error(3)];
    d_pose_error_rot = (pose_error_rot - pose_error_rot_last)./t_delta;
    pose_error_rot_last = pose_error_rot;
    heading_correction = pose_error_rot(1)*Kp_heading + d_pose_error_rot(1)*Kd_heading;
    axial_correction = pose_error_rot(2)*Kp_axial + d_pose_error_rot(2)*Kd_axial;
    cur_vel = subs(motion_profile, b, t);
    cur_dist = cur_dist +cur_vel*t_delta;
    vL = double(subs(speed - d/2*omega, u,cur_dist)*cur_vel)+heading_correction+axial_correction;
    vR = double(subs(speed + d/2*omega, u,cur_dist)*cur_vel)-heading_correction+axial_correction;
    msg = rosmessage(pub);
    msg.Data = [vL, vR];
    send(pub, msg);
% Read odo
    [msg2,status,statustext] = receive(enc,10);
    enc_delta = msg2.Data-enc_last;
    v_wheels = enc_delta/t_delta;
    v = mean(v_wheels);
    w = (v_wheels(2)-v_wheels(1))/0.235;
%     [imuMsg,status,statustext] = receive(imu,10);
%     heading = quat2eul([imuMsg.Orientation.X, imuMsg.Orientation.Y, imuMsg.Orientation.Z, imuMsg.Orientation.W]);
    if display_vel_graph
        figure(1);
        hold on;
        plot(t, cur_dist, 'r*');
        plot(t, cur_vel, 'b*');
        hold off;
    end
    if display_error_graph
        figure(3);
        hold on;
        plot(t, norm(pose_error(1:2)), 'r*');
        plot(t, pose_error(3), 'b*');
        hold off;
    end
    pose(1) = pose(1)+v(1)*cos(pose(3))*t_delta;
    pose(2) = pose(2)+v(1)*sin(pose(3))*t_delta;
    pose(3) = pose(3)+w*t_delta;
%     pose(3) = heading(3);
    if display_pos_graph
        figure (2);
        hold on
        plot(target_pose(:,1), target_pose(:,2), 'b*');
        plot(pose(:,1), pose(:,2), 'r*'); axis equal;
        hold off;
    end
    poses = [poses;pose];
    enc_last = msg2.Data;
    t_last = t;
    pause(0.005);
end
msg = rosmessage(pub);
msg.Data = [0, 0];
send(pub, msg);
