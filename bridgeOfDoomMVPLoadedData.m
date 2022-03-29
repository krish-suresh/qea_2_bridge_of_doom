function bridgeOfDoomMVP()
    % Insert any setup code you want to run here

    % u will be our parameter
    show_plots = false;
    syms u;
    beta = 5;
    t_end = beta*3.2;
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


    %experimental data
    
    load("my_data.csv");
    time = my_data(:,1);
     official_time = [];
     for i = 1:(size(time, 1)-1)
        official_time = [official_time; time(i) - 9313.8];
     end
    data_left = [];
    data_right = [];
    data_both = [];
    data_omega = [];
    for i = 1:(size(official_time, 1)-1)
        row1 = i;
        row2 = i+1;
        time1 = official_time(row1,1);
        left_wheel1 = my_data(row1,2);
        right_wheel1 = my_data(row1,3);
        time2 = official_time(row2,1);
        left_wheel2 = my_data(row2,2);
        right_wheel2 = my_data(row2,3);
        velo_left = ((left_wheel2 - left_wheel1) / (time2 - time1));
        velo_right = ((right_wheel2 - right_wheel1) / (time2 - time1));
        velo_both_wheels = (velo_left + velo_right)/2;
        omega_two = ((velo_right - velo_left) / .235);
        data_left = [data_left; velo_left];
        data_right = [data_right; velo_right];
        data_both = [data_both; velo_both_wheels];
        data_omega = [data_omega; omega_two];
    end
    new_time = official_time(1:149,:);
    d = .235;
    location = [0,0,0];
    Y = [];
    fat_holder = [0,0,0,0];
    Thfat = [];
    Nhfat = [];
    for a = 2:(size(official_time, 1)-1)
        row_one = a;
        row3 = a-1;
        time_one = official_time(row3,1);
        left = my_data(row3,2);
        right = my_data(row3,3);
        time_two = official_time(row_one,1);
        left2 = my_data(row_one,2);
        right2 = my_data(row_one,3);
        speed_left = ((left2 - left) / (time_two - time_one));
        speed_right = ((right2 - right) / (time_two - time_one));
        velo_car = (speed_left + velo_right)/2;
        omega_three = ((speed_right - speed_left) / d);
        location(1) = location(1) + velo_car*cos(location(3))*(official_time(a)-official_time(a-1));
        location(2) = location(2) + velo_car*sin(location(3))*(official_time(a)-official_time(a-1));
        location(3) = location(3) + omega_three*(official_time(a)-official_time(a-1));
        Y(a,1) = location(1) - 1.33493;
        Y(a,2) = location(2) - 3.90238;
        Y(a,3) = location(3);
    end
    for a = [3,30,60,90,120,147]
            tangent = (Y(a,1:2) - (Y(a-1,1:2))) / (official_time(a) - official_time(a-1));
            tangent_next = ((Y(a+2,1:2)) - (Y(a+1,1:2))) / (official_time(a+2) - official_time(a+1));
            Thfat_next = tangent_next/norm(tangent_next);
            Thfat = tangent/norm(tangent);
            Normal = (Thfat_next - Thfat) / (official_time(a+1) - official_time(a)); 
            Nhfat = Normal/norm(Normal);
            fat_holder(a,1) = Thfat(:,1);
            fat_holder(a,2) = Thfat(:,2);
            fat_holder(a,3) = Nhfat(:,1);
            fat_holder(a,4) = Nhfat(:,2);
     end


    d = 0.235;
    if show_plots
        figure(1)
        fplot(R(1),R(2),[0 t_end]);  hold on;
        plot(Y(:,1), Y(:,2), "--"); hold on
        for timer = [3,30,60,90,120,147]
            quiver(Y(timer,1),Y(timer,2),fat_holder(timer,1),fat_holder(timer,2))
            quiver(Y(timer,1),Y(timer,2),fat_holder(timer,3),fat_holder(timer,4))
        end
        for vector_t=1:3:t_end
                quiver(subs(R(1),u,vector_t), subs(R(2),u,vector_t), subs(That(1),u,vector_t), subs(That(2),u,vector_t));
                quiver(subs(R(1),u,vector_t), subs(R(2),u,vector_t), subs(Nhat(1),u,vector_t), subs(Nhat(2),u,vector_t));
        end
        xlabel("x (meters)")
        ylabel("y (meters)")
        legend('theoretical', 'experimental')
        axis([-4, 4, -4, 4]);
        axis padded;
        axis equal;
        hold off;

        figure(2)
        subplot(2,1,1);
        fplot(norm(T), [0 t_end]); hold on;
        plot(new_time, data_both, '--'); hold on;
        xlabel("t (seconds)")
        ylabel("speed (meters/sec)")
        legend('theoretical', 'experimental')
        %         TODO Plot exp data with dashed line

        subplot(2,1,2);
        fplot(omega, [0 t_end]); hold on;
        plot(new_time, data_omega, '--'); hold on;
        xlabel("t (seconds)")
        ylabel("omega (radians/sec)")
        legend('theoretical', 'experimental')
%         TODO Plot exp data with dashed line
        hold off;
        
        figure(3);
        fplot(norm(T) - d/2*omega, [0 t_end]); hold on
        fplot(norm(T) + d/2*omega, [0 t_end]);
        plot(new_time, data_left, '--'); hold on;
        plot(new_time, data_right, '--');
        xlabel("t (seconds)")
        ylabel("speed (meters/sec)")
        legend("Left Wheel Velocity Theoretical", "Right Wheel Velocity Theoretical", ...
            "Left Wheel Velocity Experimental", "Right Wheel Velocity Experiemntal");
        legend('Location', "northeast");
    end

    pub = rospublisher('raw_vel');
    t = 0;
%     stop the robot if it's going right now
    stopMsg = rosmessage(pub);
    stopMsg.Data = [0 0];
    send(pub, stopMsg);

    bridgeStart = double(subs(R,u,0));
    startingThat = double(subs(That,u,0));
    placeNeato(bridgeStart(1),  bridgeStart(2), startingThat(1), startingThat(2));

    % wait a bit for robot to fall onto the bridge
    pause(2);

    rostic;

    while t < t_end
        t = rostoc();
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