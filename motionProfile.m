function motionProfile(max_velocity_0, acceleration_0, time_total_0)

%Arguments
max_velocity = max_velocity_0;
acc = acceleration_0;
time_total = time_total_0;

syms b;

%Time to reach max velocity
t_1 = max_velocity / acc;
%Time to begin deceleration 
t_2 = time_total - t_1;

%Velocity piecewise function
velocity = piecewise(b < t_1, (b*acc), t_1 < b < t_2, max_velocity, b > t_2, -(b*acc)+42.5);
%Plot piecewise function
fplot(velocity, [0,time_total])

end