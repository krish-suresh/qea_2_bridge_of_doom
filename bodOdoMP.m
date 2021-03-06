
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

total_dist = vpa(int(norm(T),u ,[0, 3.2]))

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
% placeNeato(bridgeStart(1),  bridgeStart(2), startingThat(1), startingThat(2),0.5);

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
figure(2);
clf;
fplot(R(1),R(2),[0 double(total_dist)]);  hold on;
xlabel("x (meters)")
ylabel("y (meters)")
axis([-4, 4, -4, 4]);
axis padded;
axis equal;
figure(1);
clf;
xlabel("time (seconds)");

axis([0, double(t_end), 0, double(total_dist)]);
cur_dist = 0;
cur_vel = 0;
t_delta = 0;
rostic;

while true
    t = rostoc();
    if t > t_end
        break
    end
% Drive robot

    cur_vel = subs(motion_profile, b, t);
    cur_dist = cur_dist +cur_vel*t_delta;
    vL = double(subs(speed - d/2*omega, u,cur_dist)*cur_vel);
    vR = double(subs(speed + d/2*omega, u,cur_dist)*cur_vel);
    msg = rosmessage(pub);
    msg.Data = [vL, vR];
    send(pub, msg);
% Read odo
    [msg2,status,statustext] = receive(enc,10);
    t_delta = t-t_last;
    enc_delta = msg2.Data-enc_last;
    v_wheels = enc_delta/t_delta;
    v = mean(v_wheels);
    w = (v_wheels(2)-v_wheels(1))/0.235;
%     [imuMsg,status,statustext] = receive(imu,10);
%     heading = quat2eul([imuMsg.Orientation.X, imuMsg.Orientation.Y, imuMsg.Orientation.Z, imuMsg.Orientation.W]);
    figure(1);
    hold on;
    plot(t, cur_dist, 'r*');
    plot(t, cur_vel, 'b*');
    hold off;
%     plot(vR, t, 'b*');
    pose(1) = pose(1)+v(1)*cos(pose(3))*t_delta;
    pose(2) = pose(2)+v(1)*sin(pose(3))*t_delta;
    pose(3) = pose(3)+w*t_delta;
%     pose(3) = heading(3);
    figure (2);
    hold on
    plot(pose(:,1), pose(:,2), 'r*'); axis equal;
    hold off;
    poses = [poses;pose];
    enc_last = msg2.Data;
    t_last = t;
    pause(0.005);
end
msg = rosmessage(pub);
msg.Data = [0, 0];
send(pub, msg);
