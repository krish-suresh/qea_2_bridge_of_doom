function bridgeOfDoomMVP()
% Insert any setup code you want to run here

% u will be our parameter
syms u;
beta = 5;
% this is the equation of the bridge
R = 4*[0.396*cos(2.65*(u/beta+1.4));
       -0.99*sin(u/beta+1.4);
       0];

% tangent vector
T = diff(R);

% normalized tangent vector
That = T/norm(T);
N = diff(That);

% angular velocity vector
omega = cross(That, N);
omega = omega(3);

d = 0.235;


pub = rospublisher('raw_vel');

% stop the robot if it's going right now
stopMsg = rosmessage(pub);
stopMsg.Data = [0 0];
send(pub, stopMsg);

bridgeStart = double(subs(R,u,0));
startingThat = double(subs(That,u,0));
placeNeato(bridgeStart(1),  bridgeStart(2), startingThat(1), startingThat(2));

% wait a bit for robot to fall onto the bridge
pause(2);

rostic;

while true
    t = rostoc();
    
    if t > 15
        break
    end
    msg = rosmessage(pub);
    vL = double(subs(norm(T) - d/2*omega, u, t));
    vR = double(subs(norm(T) + d/2*omega, u, t));
    if vL > 3
        disp(vL)
        disp(t)
    end
    if vR > 3
        disp(vR)
        disp(t)
    end
    msg.Data = [vL, vR];
    send(pub, msg);
    pause(0.01);
end
msg = rosmessage(pub);
msg.Data = [0, 0];
send(pub, msg);
end
