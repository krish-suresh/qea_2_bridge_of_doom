pub = rospublisher('raw_vel');

%     stop the robot if it's going right now
stopMsg = rosmessage(pub);
stopMsg.Data = [0 0];
send(pub, stopMsg);

placeNeato(0,0,0,0);

enc = rossubscriber('/encoders');
clock = rossubscriber('/clock');
imu = rossubscriber('/imu');
accel = rossubscriber("/accel");

pause(2);



d = 0.235;
% [msg2,status,statustext] = receive(clock,1)
t_last = 0; % msg2.Clock_.Nsec/1e-9
[msg2,status,statustext] = receive(enc,10);
enc_last = msg2.Data;


pose = [0,0,0];
clf
plot(pose(1), pose(2), 'r*'); hold on;
rostic;
while true
    t = rostoc();
%     [msg2,status,statustext] = receive(clock,10);
%     t = msg2.Data;
    [msg2,status,statustext] = receive(enc,10);
    t_delta = t-t_last;
    enc_delta = msg2.Data-enc_last;
    v_wheels = enc_delta/t_delta;
    v = mean(v_wheels);
    omega = (v_wheels(2)-v_wheels(1))/d;
    
    [imuMsg,status,statustext] = receive(imu,10);
    heading = quat2eul([imuMsg.Orientation.X, imuMsg.Orientation.Y, imuMsg.Orientation.Z, imuMsg.Orientation.W]);

    pose(1) = pose(1)+v(1)*cos(pose(3))*t_delta;
    pose(2) = pose(2)+v(1)*sin(pose(3))*t_delta;
%     pose(3) = pose(3)+omega*t_delta;
    pose(3) = heading(3)
    pose;
    plot(pose(1), pose(2), 'r*')
    axis equal;
    enc_last = msg2.Data;
    t_last = t;
    if t > 10
        break
    end
    msg = rosmessage(pub);
    msg.Data = [0.25, 0.5];
    send(pub, msg);
    pause(0.01);
end
msg = rosmessage(pub);
msg.Data = [0, 0];
send(pub, msg);