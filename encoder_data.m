clear;
clf;
load("my_data.csv");
time = my_data(:,1);
left_wheel = my_data(:,2);
right_wheel = my_data(:,3);
hold on
plot(time, left_wheel)
plot(time, right_wheel)
xlabel('Time (sec)')
ylabel('Meters')
legend("left wheel (meters)", "right wheel (meters)")
legend('Location','northwest')