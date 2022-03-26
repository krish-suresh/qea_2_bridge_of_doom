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
% fplot(velocity, [0,time_total]);
% axis padded;
end