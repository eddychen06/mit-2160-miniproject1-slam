% Example code relevant to implementing Section 1: Odometry.
% Script-level code may require significant modification and expansion to 
% properly answer the questions of Section 1.

addpath('helper_functions', 'graphics_functions');
odom_struct = odom_reader(fullfile('data', 'odom.csv'));

%% Part a
part_a_estimate_state_init = zeros(3,1);
part_a_estimate_state_init(1) = 1.00; %x_init (m)
part_a_estimate_state_init(2) = 5.00; %y_init (m)
part_a_estimate_state_init(3) = -pi/2; %th_init (m)

part_a_estimate_covariance_init = diag([1/48, 1/48, pi^2/972]);...
    %P_init (mixed unit matrix)

%% Part b
part_b_estimate_init.state = part_a_estimate_state_init;
part_b_estimate_init.covariance = part_a_estimate_covariance_init;

part_b_time_count = odom_struct.general.time_count;

part_b_estimate_history.time = odom_struct.general.time;
part_b_estimate_history.time_count = part_b_time_count;
part_b_estimate_history.content = cell(part_b_time_count,1);

% First Time Point: No new information from data
% (just sets up first reference for delta_s)
part_b_estimate_history.content{1} = part_b_estimate_init;
wheel_prev.right = odom_struct.general.content{1}.s_right;
wheel_prev.left = odom_struct.general.content{1}.s_left;

% All other time points: new information from data, and we use it
for time_iter = 2:part_b_time_count
    % Figure out how much the wheels have turned
    wheel_next.right = odom_struct.general.content{time_iter}.s_right;
    wheel_next.left = odom_struct.general.content{time_iter}.s_left;
    wheel_turns.right = wheel_next.right - wheel_prev.right;
    wheel_turns.left = wheel_next.left - wheel_prev.left;
    wheel_prev = wheel_next;

    % Apply Kalman predict step
    part_b_estimate_history.content{time_iter} = ...
        part_b_kalman_predict(...
        part_b_estimate_history.content{time_iter-1},wheel_turns);
end

%% Part b: Post-Processing
part_b_plotting_time = part_b_estimate_history.time;
part_b_plotting_x = NaN(part_b_estimate_history.time_count,1);
part_b_plotting_y = NaN(part_b_estimate_history.time_count,1);
part_b_plotting_th = NaN(part_b_estimate_history.time_count,1);

for time_iter = 1:part_b_estimate_history.time_count
    part_b_plotting_x(time_iter) = ...
        part_b_estimate_history.content{time_iter}.state(1);
    part_b_plotting_y(time_iter) = ...
        part_b_estimate_history.content{time_iter}.state(2);
    part_b_plotting_th(time_iter) = ...
        part_b_estimate_history.content{time_iter}.state(3);
end

figure(1);clf;
plot(part_b_plotting_time,part_b_plotting_x);
xlabel('Time (s)');ylabel('x Estimate (m)');

figure(2);clf;
plot(part_b_plotting_time,part_b_plotting_y);
xlabel('Time (s)');ylabel('y Estimate (m)');

figure(3);clf;
plot(part_b_plotting_time,part_b_plotting_th);
xlabel('Time (s)');ylabel('th Estimate (rad)');

figure(4);clf;
plot(part_b_plotting_x,part_b_plotting_y);
axis equal;
xlabel('x Estimate (m)');ylabel('y Estimate (m)');

odom_plot_app(part_b_estimate_history);

cov_time = part_b_estimate_history.time(:);
pose_variances = zeros(numel(cov_time), 3);

for idx = 1:numel(cov_time)
    P = part_b_estimate_history.content{idx}.covariance;
    pose_variances(idx, :) = diag(P(1:3, 1:3)).';
end

figure('Name', 'Part (b): Odometry Pose Covariance');
tiledlayout(3, 1);

nexttile;
plot(cov_time, pose_variances(:, 1), 'LineWidth', 1.5);
ylabel('Var(x) (m^2)');
grid on;

nexttile;
plot(cov_time, pose_variances(:, 2), 'LineWidth', 1.5);
ylabel('Var(y) (m^2)');
grid on;

nexttile;
plot(cov_time, pose_variances(:, 3), 'LineWidth', 1.5);
ylabel('Var(\theta) (rad^2)');
xlabel('Time (s)');
grid on;

sgtitle('Odometry-Only Pose Covariance');