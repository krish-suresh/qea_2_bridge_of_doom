display_vel_graph = false;
display_pos_graph = false;
display_error_graph = false;
R_fun = @(theta) ([cos(theta) -sin(theta); sin(theta) cos(theta)]);
syms u b;

% this is the equation of the bridge

R = 4*[0.396*cos(2.65*(u+1.4));
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

total_dist = vpa(int(norm(T),u ,[0, 3]))/4

acc = 0.03;
max_velocity = 0.05;
t_1 = max_velocity / acc;

t_end = total_dist/max_velocity + t_1
%Time to begin deceleration 
t_2 = t_end - t_1;


%Velocity piecewise function
motion_profile = piecewise(b < t_1, (b*acc), t_1 < b < t_2, max_velocity, b > t_2, max_velocity-(b-t_2)*acc);
distance_traveled = int(motion_profile, b);
pub = rospublisher('raw_vel');
enc = rossubscriber('/encoders');
sub_states = rossubscriber('/gazebo/model_states', 'gazebo_msgs/ModelStates');
imu = rossubscriber('/imu');
% accel = rossubscriber("/accel");
%     stop the robot if it's going right now
stopMsg = rosmessage(pub);
stopMsg.Data = [0 0];
send(pub, stopMsg);

bridgeStart = double(subs(R,u,0));
startingThat = double(subs(That,u,0));
placeNeato(bridgeStart(1),  bridgeStart(2), startingThat(1), startingThat(2),0.6);

% wait a bit for robot to fall onto the bridge
pause(2);
d = 0.235;
t_last = 0;
[msg2,status,statustext] = receive(enc,10);
enc_last = msg2.Data;
[imuMsg,status,statustext] = receive(imu,10);
heading = quat2eul([imuMsg.Orientation.X, imuMsg.Orientation.Y, imuMsg.Orientation.Z, imuMsg.Orientation.W]);


pose = [bridgeStart(1),  bridgeStart(2),heading(3)];
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
    cur_vel = subs(motion_profile, b, t);
    cur_dist = cur_dist +cur_vel*t_delta;
    bridgeStart = double(subs(R,u,cur_dist));
    startingThat = double(subs(That,u,cur_dist));
    placeNeato(bridgeStart(1),  bridgeStart(2), startingThat(1), startingThat(2),0.51);
    t_last = t;
end
msg = rosmessage(pub);
msg.Data = [0, 0];
send(pub, msg);
