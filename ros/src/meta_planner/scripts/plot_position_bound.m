%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Plot planner/tracker position and tracking bound, in each dimension.
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Read all the data.
planner_xs = csvread('planner_xs.csv');
planner_ys = csvread('planner_ys.csv');
planner_zs = csvread('planner_zs.csv');

tracker_xs = csvread('tracker_xs.csv');
tracker_ys = csvread('tracker_ys.csv');
tracker_zs = csvread('tracker_zs.csv');

bound_xs = csvread('bound_xs.csv');
bound_ys = csvread('bound_ys.csv');
bound_zs = csvread('bound_zs.csv');

times = csvread('times.csv');

FIRST_TIME = 0.2;
LAST_TIME = 10.0;
indices = (times > FIRST_TIME) & (times < LAST_TIME);

%% Plot subfigures, sharing the same x-axis (time).
FONT_SIZE = 24;
LINE_WIDTH = 2;

figure;

subplot(3, 1, 1);
set(gca, 'fontsize', FONT_SIZE);
grid on;
hold on;
plot(times(indices), planner_xs(indices), 'k', 'LineWidth', LINE_WIDTH);
plot(times(indices), tracker_xs(indices), 'b--', 'LineWidth', LINE_WIDTH);
plot(times(indices), planner_xs(indices) + bound_xs(indices), 'r:', 'LineWidth', LINE_WIDTH);
plot(times(indices), planner_xs(indices) - bound_xs(indices), 'r:', 'LineWidth', LINE_WIDTH);
hold off;
legend('Planner', 'Tracker', 'TEB');
ylabel('$x$ (m)', 'Interpreter', 'latex');

subplot(3, 1, 2);
set(gca, 'fontsize', FONT_SIZE);
hold on; grid on;
plot(times(indices), planner_ys(indices), 'k', 'LineWidth', LINE_WIDTH);
plot(times(indices), tracker_ys(indices), 'b--', 'LineWidth', LINE_WIDTH);
plot(times(indices), planner_ys(indices) + bound_ys(indices), 'r:', 'LineWidth', LINE_WIDTH);
plot(times(indices), planner_ys(indices) - bound_ys(indices), 'r:', 'LineWidth', LINE_WIDTH);
hold off; 
ylabel('$y$ (m)', 'Interpreter', 'latex');

subplot(3, 1, 3);
set(gca, 'fontsize', FONT_SIZE);
hold on; grid on;
plot(times(indices), planner_zs(indices), 'k', 'LineWidth', LINE_WIDTH);
plot(times(indices), tracker_zs(indices), 'b--', 'LineWidth', LINE_WIDTH);
plot(times(indices), planner_zs(indices) + bound_zs(indices), 'r:', 'LineWidth', LINE_WIDTH);
plot(times(indices), planner_zs(indices) - bound_zs(indices), 'r:', 'LineWidth', LINE_WIDTH);
hold off; 
ylabel('$z$ (m)', 'Interpreter', 'latex');
xlabel('Time (s)');