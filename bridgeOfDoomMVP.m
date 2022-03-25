function bridgeOfDoomMVP()
    % Insert any setup code you want to run here

    % u will be our parameter
    show_plots = false
    syms u;
    beta = 5;
    t_end = 15;
    % this is the equation of the bridge
    R = 4*[0.396*cos(2.65*(u/beta+1.4));
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

    d = 0.235;
    if show_plots
        figure(3)
        fplot(R(1),R(2),[0 t_end]);  hold on;
        for vector_t=1:3:t_end
                quiver(subs(R(1),u,vector_t), subs(R(2),u,vector_t), subs(That(1),u,vector_t), subs(That(2),u,vector_t));
                quiver(subs(R(1),u,vector_t), subs(R(2),u,vector_t), subs(Nhat(1),u,vector_t), subs(Nhat(2),u,vector_t));
        end
        xlabel("x (meters)")
        ylabel("y (meters)")
        axis([-4, 4, -4, 4]);
        axis padded;
        axis equal;
        hold off;

        figure(1)
        fplot(norm(T), [0 t_end]);
        xlabel("t (seconds)")
        ylabel("speed (meters/sec)")

        figure(2)
        fplot(omega, [0 t_end]);
        xlabel("t (seconds)")
        ylabel("omega (radians/sec)")
    end

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
        
        if t > t_end
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
