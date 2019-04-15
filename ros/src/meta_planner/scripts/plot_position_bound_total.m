%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Plot planner/tracker position and tracking bound, in each dimension.
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Read all the data.
directory = '../../neural_tracker/data/run2/';
planner_xs = csvread(strcat(directory, 'planner_xs.csv'));
planner_ys = csvread(strcat(directory, 'planner_ys.csv'));
planner_zs = csvread(strcat(directory, 'planner_zs.csv'));

tracker_xs = csvread(strcat(directory, 'tracker_xs.csv'));
tracker_ys = csvread(strcat(directory, 'tracker_ys.csv'));
tracker_zs = csvread(strcat(directory, 'tracker_zs.csv'));

bound_xs = csvread(strcat(directory, 'bound_xs.csv'));
bound_ys = csvread(strcat(directory, 'bound_ys.csv'));
bound_zs = csvread(strcat(directory, 'bound_zs.csv'));

times = csvread(strcat(directory, 'times.csv'));

% Extract indices.
FIRST_TIME = 0.2;
LAST_TIME = 10.0;
indices = (times > FIRST_TIME) & (times < LAST_TIME);

% Get relative L2 distance.
error = sqrt((planner_xs(indices) - tracker_xs(indices)).^2 + ...
    (planner_ys(indices) - tracker_ys(indices)).^2 + ...
    (planner_zs(indices) - tracker_zs(indices)).^2);

%% Plot subfigures, sharing the same x-axis (time).
FONT_SIZE = 28;
LINE_WIDTH = 4;

figure;
set(gca, 'fontsize', FONT_SIZE);
set(gcf, 'color', 'white');
grid on;
hold on;
plot(times(indices), error, 'k', 'LineWidth', LINE_WIDTH);
plot(times(indices), bound_xs(indices), 'r:', 'LineWidth', LINE_WIDTH);
hold off;
legend('Tracking error', 'TEB');
ylabel('Relative distance (m)', 'Interpreter', 'latex');
xlabel('Time (s)', 'Interpreter', 'latex');