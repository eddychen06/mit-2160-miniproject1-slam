% Example code relevant to implementing Section 2: SLAM.
% Script-level code may require significant modification and expansion to 
% properly answer the questions of Section 2.

addpath('helper_functions', 'graphics_functions');

odom_struct = odom_reader(fullfile('data', 'odom.csv'));

scan_struct = scan_reader( ...
    fullfile('data', 'scan_dist.csv'), ...
    fullfile('data', 'scan_partition.csv'), ...
    fullfile('data', 'scan_partition_labels.csv'));

data_struct = data_interleaver(odom_struct, scan_struct);

%% Part d
part_d_estimate_state_init = [1;5;-pi/2;... %x, y, th
    0;pi/2;...%r^1_map, alpha^1_map
    5;0;...%r^2_map, alpha^2_map
    4;pi/2;...%r^3_map, alpha^3_map
    2;0;...%r^4_map, alpha^4_map
    6;pi/2;...%r^5_map, alpha^5_map
    0;0;...%r^6_map, alpha^6_map
    1;pi/2;...%r^7_map, alpha^7_map
    4;0;...%r^8_map, alpha^8_map
    1.5;pi/2;...%r^9_map, alpha^9_map
    3.5;0 ...%r^10_map, alpha^10_map
    ];

part_d_estimate_covariance_init = diag([1/48,1/48,pi^2/972,... %x, y, th
    0,0,...%r^1_map, alpha^1_map
    0.01^2,0.0283^2,...%r^2_map, alpha^2_map
    0.01^2,0.0283^2,...%r^3_map, alpha^3_map
    0.01^2,0.0283^2,...%r^4_map, alpha^4_map
    0.01^2,0.0283^2,...%r^5_map, alpha^5_map
    0,0.0283^2,...%r^6_map, alpha^6_map
    0.01^2,0.0283^2,...%r^7_map, alpha^7_map
    0.01^2,0.0283^2,...%r^8_map, alpha^8_map
    0.01^2,0.0283^2,...%r^9_map, alpha^9_map
    0.01^2,0.0283^2 ...%r^10_map, alpha^10_map
    ]);

%% Part h
% Manual Single-Scan Plot
part_h_processed_scan_history.time = scan_struct.general.time;
part_h_processed_scan_history.time_count = scan_struct.general.time_count;
part_h_processed_scan_history.content = ...
    cell(scan_struct.general.time_count,1);

for scan_iter = 1:(part_h_processed_scan_history.time_count)
    part_h_processed_scan_history.content{scan_iter} = ...
        part_h_full_scan_fitter(scan_struct.general.content{scan_iter});
end

% Scan Plot Application
scan_plot_app(part_h_processed_scan_history,scan_struct.raw_scan);

%% Part k
part_k_estimate_init.state = part_d_estimate_state_init;
part_k_estimate_init.covariance = part_d_estimate_covariance_init;

part_k_time_count = data_struct.general.time_count;

part_k_estimate_history.time = data_struct.general.time;
part_k_estimate_history.time_count = part_k_time_count;
part_k_estimate_history.content = cell(part_k_time_count,1);

part_k_processed_scan_history.time = scan_struct.general.time;
part_k_processed_scan_history.time_count = scan_struct.general.time_count;
part_k_processed_scan_history.content = ...
    cell(scan_struct.general.time_count,1);

scan_number = 0;

% First Time Point: No new information from data
% (just sets up first reference for delta_s)
part_k_estimate_history.content{1} = part_k_estimate_init;
wheel_prev.right = data_struct.general.content{1}.s_right;
wheel_prev.left = data_struct.general.content{1}.s_left;

% All other time points: new information from data, and we use it
for time_iter = 2:part_k_time_count
    data_type = data_struct.general.content{time_iter}.type;
    if isequal(data_type,'odom')
        % Figure out how much the wheels have turned
        wheel_next.right = data_struct.general.content{time_iter}.s_right;
        wheel_next.left = data_struct.general.content{time_iter}.s_left;
        wheel_turns.right = wheel_next.right - wheel_prev.right;
        wheel_turns.left = wheel_next.left - wheel_prev.left;
        wheel_prev = wheel_next;
    
        % Apply Kalman predict step
        part_k_estimate_history.content{time_iter} = ...
            part_b_kalman_predict(...
            part_k_estimate_history.content{time_iter-1},wheel_turns);
    elseif isequal(data_type,'scan')
        scan_number = scan_number + 1;
        scan_now = data_struct.general.content{time_iter};
        [part_k_estimate_history.content{time_iter},...
            part_k_processed_scan_history.content{scan_number}] = ...
            part_k_kalman_update(...
            part_k_estimate_history.content{time_iter-1},scan_now);
    end
end

%% Part k Post-Processing
wall_list = {'Wall 1','Wall 2','Wall 3','Wall 4','Wall 5','Wall 6',...
    'Wall 7','Wall 8','Wall 9','Wall 10'};

part_k_plotting_time = part_k_estimate_history.time;
part_k_plotting_x = NaN(part_k_estimate_history.time_count,1);
part_k_plotting_y = NaN(part_k_estimate_history.time_count,1);
part_k_plotting_th = NaN(part_k_estimate_history.time_count,1);
part_k_plotting_pose_trace = NaN(part_k_estimate_history.time_count,1);
part_k_plotting_r_var = NaN(part_k_estimate_history.time_count,10);
part_k_plotting_alpha_var = NaN(part_k_estimate_history.time_count,10);

for time_iter = 1:part_k_estimate_history.time_count
    part_k_plotting_x(time_iter) = ...
        part_k_estimate_history.content{time_iter}.state(1);
    part_k_plotting_y(time_iter) = ...
        part_k_estimate_history.content{time_iter}.state(2);
    part_k_plotting_th(time_iter) = ...
        part_k_estimate_history.content{time_iter}.state(3);
    part_k_plotting_pose_trace(time_iter) = ...
        trace(part_k_estimate_history.content{time_iter}...
        .covariance(1:3,1:3));
    for wall_id = 1:10
        part_k_plotting_r_var(time_iter,wall_id) = ...
            part_k_estimate_history.content{time_iter}...
            .covariance(3+2*(wall_id-1)+1,3+2*(wall_id-1)+1);
        part_k_plotting_alpha_var(time_iter,wall_id) = ...
            part_k_estimate_history.content{time_iter}...
            .covariance(3+2*(wall_id-1)+2,3+2*(wall_id-1)+2);
    end
end

figure(11);clf;
plot(part_k_plotting_time,part_k_plotting_x);
xlabel('Time (s)');ylabel('x Estimate (m)');

figure(12);clf;
plot(part_k_plotting_time,part_k_plotting_y);
xlabel('Time (s)');ylabel('y Estimate (m)');

figure(13);clf;
plot(part_k_plotting_time,part_k_plotting_th);
xlabel('Time (s)');ylabel('th Estimate (rad)');

figure(14);clf;
plot(part_k_plotting_x,part_k_plotting_y);
axis equal;
xlabel('x Estimate (m)');ylabel('y Estimate (m)');

figure(15);clf;
plot(part_k_plotting_time,part_k_plotting_pose_trace);
xlabel('Time (s)');ylabel('Pose Covariance Trace');

figure(16);clf;
plot(part_k_plotting_time,sqrt(part_k_plotting_r_var));
xlabel('Time (s)');ylabel('Wall r Standard Deviation (m)');
legend(wall_list);

figure(17);clf;
plot(part_k_plotting_time,sqrt(part_k_plotting_alpha_var));
xlabel('Time (s)');ylabel('Wall alpha Standard Deviation (rad)');
legend(wall_list);

SLAM_plot_app(part_k_estimate_history,part_k_processed_scan_history,...
    scan_struct.raw_scan);

%% Part l
slam_path = zeros(part_k_estimate_history.time_count, 2);
odom_path = zeros(part_b_estimate_history.time_count, 2);

for it = 1:part_k_estimate_history.time_count
    slam_path(it,:) = ...
        part_k_estimate_history.content{it}.state(1:2).';
end

for it = 1:part_b_estimate_history.time_count
    odom_path(it,:) = ...
        part_b_estimate_history.content{it}.state(1:2).';
end

final_state = part_k_estimate_history.content{end}.state;
wall_segments = wall_line_segments_from_extended_state(final_state);

figure;
hold on;

for wall_id = 1:10
    wall_plot = plot( ...
        wall_segments(wall_id,[1,3]), ...
        wall_segments(wall_id,[2,4]), 'k-', 'LineWidth', 1);
end

odom_plot = plot(odom_path(:,1), odom_path(:,2), 'r--');
slam_plot = plot(slam_path(:,1), slam_path(:,2), 'b-');

axis equal;
xlim([-0.2, 5.3]);
ylim([-0.2, 6.3]);
xlabel('x (m)');
ylabel('y (m)');
title('SLAM and Odometry Trajectories');
legend([wall_plot, odom_plot, slam_plot], ...
    'Final estimated walls', 'Odometry only', 'SLAM');
