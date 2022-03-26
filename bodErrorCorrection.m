
pid_h = pid(1,0,0);
pid_d = pid(1,0,0);
show_plots = false;
syms u;
beta = 4;
t_end = beta*3.2;
% this is the equation of the bridge
R = [0.396*cos(2.65*(u/beta+1.4));
    -0.99*sin(u/beta+1.4);
    0];

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

pub = rospublisher('raw_vel');
enc = rossubscriber('/encoders');
% clock = rossubscriber('/clock');
imu = rossubscriber('/imu');
% accel = rossubscriber("/accel");
%     stop the robot if it's going right now
stopMsg = rosmessage(pub);
stopMsg.Data = [0 0];
send(pub, stopMsg);

bridgeStart = double(subs(R,u,0));
startingThat = double(subs(That,u,0));
placeNeato(bridgeStart(1),  bridgeStart(2), startingThat(1), startingThat(2),0.5);

% wait a bit for robot to fall onto the bridge
pause(2);
d = 0.235;
% [msg2,status,statustext] = receive(clock,1)
t_last = 0; % msg2.Clock_.Nsec/1e-9
[msg2,status,statustext] = receive(enc,10);
enc_last = msg2.Data;
[imuMsg,status,statustext] = receive(imu,10);
heading = quat2eul([imuMsg.Orientation.X, imuMsg.Orientation.Y, imuMsg.Orientation.Z, imuMsg.Orientation.W]);


pose = [bridgeStart(1),  bridgeStart(2),heading(3)];
poses = pose;
figure(1);
clf
fplot(R(1),R(2),[0 t_end]);  hold on;
xlabel("x (meters)")
ylabel("y (meters)")
axis([-4, 4, -4, 4]);
axis padded;
axis equal;
plot(pose(1), pose(2), 'r*'); hold on;
rostic;

while true
    t = rostoc();
% Drive robot
    msg = rosmessage(pub);
    vL = double(subs(speed - d/2*omega, u, t));
    vR = double(subs(speed + d/2*omega, u, t));
    msg.Data = [vL, vR];
    send(pub, msg);
% Read odo
    [msg2,status,statustext] = receive(enc,10);
    t_delta = t-t_last;
    enc_delta = msg2.Data-enc_last;
    v_wheels = enc_delta/t_delta;
    v = mean(v_wheels);
    w = (v_wheels(2)-v_wheels(1))/0.235;
    [imuMsg,status,statustext] = receive(imu,10);
    heading = quat2eul([imuMsg.Orientation.X, imuMsg.Orientation.Y, imuMsg.Orientation.Z, imuMsg.Orientation.W]);

    pose(1) = pose(1)+v(1)*cos(pose(3))*t_delta;
    pose(2) = pose(2)+v(1)*sin(pose(3))*t_delta;
%     pose(3) = pose(3)+w*t_delta;
    pose(3) = heading(3);
    plot(pose(:,1), pose(:,2), 'r*'); axis equal;

    poses = [poses;pose];
    enc_last = msg2.Data;
    t_last = t;

    if t > t_end
        break
    end

    pause(0.005);
end
msg = rosmessage(pub);
msg.Data = [0, 0];
send(pub, msg);
