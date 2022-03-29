function [velocity, time_total] = motionProfile(max_velocity, acc, distance)

syms b;

%Time to reach max velocity
t_1 = max_velocity / acc

time_total = distance/max_velocity + t_1
%Time to begin deceleration 
t_2 = time_total - t_1


%Velocity piecewise function
velocity = piecewise(b < t_1, (b*acc), t_1 < b < t_2, max_velocity, b > t_2, max_velocity-(b-t_2)*acc);
%Plot piecewise function
pos = 0;
del = 0.01;
poses = []
for i=0:del:time_total
    v = subs(velocity, b, i);
    if ~isnan(v)
        pos = pos+del*double(v);
        poses = [poses, pos];
    else
        poses = [poses, pos];
    end
%     plot(i, pos, "r.");
end 
plot(0:del:time_total, poses); hold on;
fplot(velocity, [0,time_total]); 
accel = [0,acc;t_1, acc;t_1, 0; t_2,0; t_2, -acc; time_total, -acc]
plot(accel(:,1), accel(:,2));
legend({"Position","Velocity",  "Acceleration"}, 'Location', "northwest")
axis padded;
end