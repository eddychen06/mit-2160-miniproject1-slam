function SLAM_plot_app(part_k_estimate_history,...
    part_k_processed_scan_history,raw_scan)
%SLAM_PLOT_APP This generates an interactive visualization showing the
%SLAM-estimated trajectory of the robot up to a certain point in time,
%as well as the robot's current pose estimate at that time, the
% (x,y) covariance of that estimate at that time, the current wall estimate
% at that time, and the current wall estimate standard deviations at that
% time.  The pose covariance is illustrated using a Z-score isoline of the
% Gaussian distribution.
%
% Also shown are the most recent raw LIDAR scan before that time, as well
% as the pre-fit-pruned, fit, and post-fit-pruned fitted wall data derived
% from that scan.
%
% The time of plotting and the Z-score for covariance plotting can both be
% changed using slider/text-box inputs at the top of the screen.

%% UI Setup
% Create UI Figure Window
fig = uifigure;
fig.Name = "SLAM Animation";

% App Layout
gl = uigridlayout(fig,[4, 4]);
gl.RowHeight = {40, 40, '1x', '1x'};
gl.ColumnWidth = {'fit', 60, '1x', '1x'};

% Create and Position UI Components
lbl_time = uilabel(gl);
txt_time = uitextarea(gl);
sld_time = uislider(gl);

lbl_z = uilabel(gl);
txt_z = uitextarea(gl);
sld_z = uislider(gl);

ax = uiaxes(gl);
pax = polaraxes(gl);
pax_2 = polaraxes(gl);

%- Position Time Label
lbl_time.Layout.Row = 1;
lbl_time.Layout.Column = 1;

%- Position Time Text Field
txt_time.Layout.Row = 1;
txt_time.Layout.Column = 2;

%- Position Time Slider
sld_time.Layout.Row = 1;
sld_time.Layout.Column = [3 4];

%- Position Z-Score Label
lbl_z.Layout.Row = 2;
lbl_z.Layout.Column = 1;

%- Position Z-Score Text Field
txt_z.Layout.Row = 2;
txt_z.Layout.Column = 2;

%- Position Z-Score Slider
sld_z.Layout.Row = 2;
sld_z.Layout.Column = [3 4];

%- Position Axes
ax.Layout.Row = [3 4];
ax.Layout.Column = [1 3];

%- Position Polar Axes
pax.Layout.Row = 3;
pax.Layout.Column = 4;

%- Position Polar Axes 2
pax_2.Layout.Row = 4;
pax_2.Layout.Column = 4;

%% Unpack Estimate History
est_time = part_k_estimate_history.time;
est_content = part_k_estimate_history.content;
x = NaN(part_k_estimate_history.time_count,1);
y = NaN(part_k_estimate_history.time_count,1);
th = NaN(part_k_estimate_history.time_count,1);
r_var = NaN(part_k_estimate_history.time_count,10);
alpha_var = NaN(part_k_estimate_history.time_count,10);
for time_iter = 1:part_k_estimate_history.time_count
    x(time_iter) = ...
        est_content{time_iter}.state(1);
    y(time_iter) = ...
        est_content{time_iter}.state(2);
    th(time_iter) = ...
        est_content{time_iter}.state(3);
    for wall_id = 1:10
        r_var(time_iter,wall_id) = est_content{time_iter}...
            .covariance(3+2*(wall_id-1)+1,3+2*(wall_id-1)+1);
        alpha_var(time_iter,wall_id) = est_content{time_iter}...
            .covariance(3+2*(wall_id-1)+2,3+2*(wall_id-1)+2);
    end
end

scan_time = part_k_processed_scan_history.time;
scan_content = part_k_processed_scan_history.content;

%% Measure Plotting Bounds for Wall Standard Deviations
r_std = sqrt(r_var);
alpha_std = sqrt(alpha_var);

r_std_range = NaN(1,2);
alpha_std_range = NaN(1,2);
r_std_range(1) = min(r_std + 1e10.*(r_std==0),[],"all");
r_std_range(2) = max(r_std,[],"all");
alpha_std_range(1) = min(alpha_std,[],"all");
alpha_std_range(2) = max(alpha_std,[],"all");

%% Configure UI Component Appearance
lbl_time.Text = "Time:";

txt_time.Value = num2str(0,'%.2f');

sld_time.Limits = [est_time(1), est_time(end)];
sld_time.Value = est_time(1);

lbl_z.Text = "Z-Score:";

txt_z.Value = num2str(4,'%.2f');

sld_z.Limits = [0.1, 5];
sld_z.Value = 4;

%% Initialize Plot
% Retrieve Robot Position
x_robot = x(1);
y_robot = y(1);
th_robot = th(1);

% Replot Trajectory
ODOM_replot_up_to_time(ax,est_time,x,y,th,1);
hold(ax,'on');

% Draw Robot
draw_robot_on_axes(ax,x_robot,y_robot,th_robot);

% Plot Covariance Matrix
plot_2d_covariance_matrix_on_axes(ax,est_content{1}.covariance(1:2,1:2),...
    [x_robot; y_robot],sld_z.Value,0.5,'r');

% Draw Walls
draw_walls_on_axes(ax,est_content{1},r_std_range,alpha_std_range);

% Draw Fitted Scan Plot
part_h_fitted_scan_plotter(pax,scan_content{1});

% Draw Raw Scan Plot
plot_raw_scan(pax_2,raw_scan,1);

%% Assign Callback Functions
sld_time.ValueChangedFcn = {@ODOM_changed_time_via_slider,...
    ax,pax,pax_2,txt_time,sld_z,est_time,x,y,th,est_content,r_std_range,alpha_std_range,scan_time,scan_content,raw_scan};
sld_time.ValueChangingFcn = {@ODOM_changing_time_via_slider,...
    ax,pax,pax_2,txt_time,sld_z,est_time,x,y,th,est_content,r_std_range,alpha_std_range,scan_time,scan_content,raw_scan};
txt_time.ValueChangedFcn = {@ODOM_changed_time_via_text,...
    ax,pax,pax_2,sld_time,sld_z,est_time,x,y,th,est_content,r_std_range,alpha_std_range,scan_time,scan_content,raw_scan};
sld_z.ValueChangedFcn = {@ODOM_changed_z_via_slider,...
    ax,pax,pax_2,sld_time,txt_z,est_time,x,y,th,est_content,r_std_range,alpha_std_range,scan_time,scan_content,raw_scan};
sld_z.ValueChangingFcn = {@ODOM_changing_z_via_slider,...
    ax,pax,pax_2,sld_time,txt_z,est_time,x,y,th,est_content,r_std_range,alpha_std_range,scan_time,scan_content,raw_scan};
txt_z.ValueChangedFcn = {@ODOM_changed_z_via_text,...
    ax,pax,pax_2,sld_time,sld_z,est_time,x,y,th,est_content,r_std_range,alpha_std_range,scan_time,scan_content,raw_scan};

end

function ODOM_changed_time_via_text(src,event,ax,pax,pax_2,sld_time,sld_z,est_time,x,y,th,est_content,r_std_range,alpha_std_range,scan_time,scan_content,raw_scan)

% Retrieve Time and Round to Nearest Actual Value
raw_time = str2double(event.Value);
[plot_time,plot_time_loc] = find_closest_element(raw_time, est_time);

% Retrieve Robot Position
x_robot = x(plot_time_loc);
y_robot = y(plot_time_loc);
th_robot = th(plot_time_loc);

% Replot Trajectory
ODOM_replot_up_to_time(ax,est_time,x,y,th,plot_time_loc);
hold(ax,'on');

% Draw Robot
draw_robot_on_axes(ax,x_robot,y_robot,th_robot);

% Plot Covariance Matrix
plot_2d_covariance_matrix_on_axes(ax,est_content{plot_time_loc}.covariance(1:2,1:2),...
    [x_robot; y_robot],sld_z.Value,0.5,'r');

% Draw Walls
draw_walls_on_axes(ax,est_content{plot_time_loc},r_std_range,alpha_std_range);

% Update Fitted Scan Plot
[scan_time,scan_time_loc] = find_previous_element(plot_time,scan_time);
cla(pax);
part_h_fitted_scan_plotter(pax,scan_content{scan_time_loc});
% Replot Raw Scan
cla(pax_2);
plot_raw_scan(pax_2,raw_scan,scan_time_loc);

% Correct Number on Slider to be on Time Axis
sld_time.Value = plot_time;
% Correct Number on Text to be on Time Axis
src.Value = num2str(plot_time,'%.2f');

end

function ODOM_changed_time_via_slider(src,event,ax,pax,pax_2,txt_time,sld_z,est_time,x,y,th,est_content,r_std_range,alpha_std_range,scan_time,scan_content,raw_scan)

% Retrieve Time and Round to Nearest Actual Value
raw_time = event.Value;
[plot_time,plot_time_loc] = find_closest_element(raw_time, est_time);

% Retrieve Robot Position
x_robot = x(plot_time_loc);
y_robot = y(plot_time_loc);
th_robot = th(plot_time_loc);

% Replot Trajectory
ODOM_replot_up_to_time(ax,est_time,x,y,th,plot_time_loc);

% Draw Robot
draw_robot_on_axes(ax,x_robot,y_robot,th_robot);

% Plot Covariance Matrix
hold(ax,'on');
plot_2d_covariance_matrix_on_axes(ax,est_content{plot_time_loc}.covariance(1:2,1:2),...
    [x_robot; y_robot],sld_z.Value,0.5,'r');

% Draw Walls
draw_walls_on_axes(ax,est_content{plot_time_loc},r_std_range,alpha_std_range);

% Update Fitted Scan Plot
[scan_time,scan_time_loc] = find_previous_element(plot_time,scan_time);
cla(pax);
part_h_fitted_scan_plotter(pax,scan_content{scan_time_loc});
% Replot Raw Scan
cla(pax_2);
plot_raw_scan(pax_2,raw_scan,scan_time_loc);

% Correct Number on Slider to be on Time Axis
src.Value = plot_time;
% Correct Number on Text to be on Time Axis
txt_time.Value = num2str(plot_time,'%.2f');

end


function ODOM_changing_time_via_slider(src,event,ax,pax,pax_2,txt_time,sld_z,est_time,x,y,th,est_content,r_std_range,alpha_std_range,scan_time,scan_content,raw_scan)

% Retrieve Time and Round to Nearest Actual Value
raw_time = event.Value;
[plot_time,plot_time_loc] = find_closest_element(raw_time, est_time);

% Retrieve Robot Position
x_robot = x(plot_time_loc);
y_robot = y(plot_time_loc);
th_robot = th(plot_time_loc);

% Replot Trajectory
ODOM_replot_up_to_time(ax,est_time,x,y,th,plot_time_loc);

% Draw Robot
draw_robot_on_axes(ax,x_robot,y_robot,th_robot);

% Plot Covariance Matrix
hold(ax,'on');
plot_2d_covariance_matrix_on_axes(ax,est_content{plot_time_loc}.covariance(1:2,1:2),...
    [x_robot; y_robot],sld_z.Value,0.5,'r');

% Draw Walls
draw_walls_on_axes(ax,est_content{plot_time_loc},r_std_range,alpha_std_range);

% Update Fitted Scan Plot
[scan_time,scan_time_loc] = find_previous_element(plot_time,scan_time);
cla(pax);
part_h_fitted_scan_plotter(pax,scan_content{scan_time_loc});
% Replot Raw Scan
cla(pax_2);
plot_raw_scan(pax_2,raw_scan,scan_time_loc);

% Correct Number on Text to be on Time Axis
txt_time.Value = num2str(plot_time,'%.2f');

end


function ODOM_changed_z_via_text(src,event,ax,pax,pax_2,sld_time,sld_z,est_time,x,y,th,est_content,r_std_range,alpha_std_range,scan_time,scan_content,raw_scan)

% Retrieve Time and Round to Nearest Actual Value
raw_time = sld_time.Value;
[plot_time,plot_time_loc] = find_closest_element(raw_time, est_time);

% Retrieve Robot Position
x_robot = x(plot_time_loc);
y_robot = y(plot_time_loc);
th_robot = th(plot_time_loc);

% Replot Trajectory
ODOM_replot_up_to_time(ax,est_time,x,y,th,plot_time_loc);

% Draw Robot
draw_robot_on_axes(ax,x_robot,y_robot,th_robot);

% Threshold z
new_z = min(max(str2double(event.Value),sld_z.Limits(1)),sld_z.Limits(2));

% Plot Covariance Matrix
hold(ax,'on');
plot_2d_covariance_matrix_on_axes(ax,est_content{plot_time_loc}.covariance(1:2,1:2),...
    [x_robot; y_robot],new_z,0.5,'r');

% Draw Walls
draw_walls_on_axes(ax,est_content{plot_time_loc},r_std_range,alpha_std_range);

% Update Fitted Scan Plot
[scan_time,scan_time_loc] = find_previous_element(plot_time,scan_time);
cla(pax);
part_h_fitted_scan_plotter(pax,scan_content{scan_time_loc});
% Replot Raw Scan
cla(pax_2);
plot_raw_scan(pax_2,raw_scan,scan_time_loc);

% Correct Number on Slider
sld_z.Value = new_z;
% Correct Number on Text
src.Value = num2str(new_z);

end

function ODOM_changed_z_via_slider(src,event,ax,pax,pax_2,sld_time,txt_z,est_time,x,y,th,est_content,r_std_range,alpha_std_range,scan_time,scan_content,raw_scan)

% Retrieve Time and Round to Nearest Actual Value
raw_time = sld_time.Value;
[plot_time,plot_time_loc] = find_closest_element(raw_time, est_time);

% Retrieve Robot Position
x_robot = x(plot_time_loc);
y_robot = y(plot_time_loc);
th_robot = th(plot_time_loc);

% Replot Trajectory
ODOM_replot_up_to_time(ax,est_time,x,y,th,plot_time_loc);

% Draw Robot
draw_robot_on_axes(ax,x_robot,y_robot,th_robot);

% Plot Covariance Matrix
hold(ax,'on');
plot_2d_covariance_matrix_on_axes(ax,est_content{plot_time_loc}.covariance(1:2,1:2),...
    [x_robot; y_robot],event.Value,0.5,'r');

% Draw Walls
draw_walls_on_axes(ax,est_content{plot_time_loc},r_std_range,alpha_std_range);

% Update Fitted Scan Plot
[scan_time,scan_time_loc] = find_previous_element(plot_time,scan_time);
cla(pax);
part_h_fitted_scan_plotter(pax,scan_content{scan_time_loc});
% Replot Raw Scan
cla(pax_2);
plot_raw_scan(pax_2,raw_scan,scan_time_loc);

% Correct Number on Text to be on Time Axis
txt_z.Value = num2str(event.Value);

end


function ODOM_changing_z_via_slider(src,event,ax,pax,pax_2,sld_time,txt_z,est_time,x,y,th,est_content,r_std_range,alpha_std_range,scan_time,scan_content,raw_scan)

% Retrieve Time and Round to Nearest Actual Value
raw_time = sld_time.Value;
[plot_time,plot_time_loc] = find_closest_element(raw_time, est_time);

% Retrieve Robot Position
x_robot = x(plot_time_loc);
y_robot = y(plot_time_loc);
th_robot = th(plot_time_loc);

% Replot Trajectory
ODOM_replot_up_to_time(ax,est_time,x,y,th,plot_time_loc);

% Draw Robot
draw_robot_on_axes(ax,x_robot,y_robot,th_robot);

% Plot Covariance Matrix
hold(ax,'on');
plot_2d_covariance_matrix_on_axes(ax,est_content{plot_time_loc}.covariance(1:2,1:2),...
    [x_robot; y_robot],event.Value,0.5,'r');

% Draw Walls
draw_walls_on_axes(ax,est_content{plot_time_loc},r_std_range,alpha_std_range);

% Update Fitted Scan Plot
[scan_time,scan_time_loc] = find_previous_element(plot_time,scan_time);
cla(pax);
part_h_fitted_scan_plotter(pax,scan_content{scan_time_loc});
% Replot Raw Scan
cla(pax_2);
plot_raw_scan(pax_2,raw_scan,scan_time_loc);

% Correct Number on Text to be on Time Axis
txt_z.Value = num2str(event.Value);

end


function ODOM_replot_up_to_time(ax,est_time,x,y,th,time_loc)

% Identify Trajectories up to Now
x_traj = x(1:time_loc);
y_traj = y(1:time_loc);
th_traj = th(1:time_loc);

cla(ax);
plot(ax,x_traj,y_traj,'k-');
axis(ax,'equal');
xlim(ax,[-0.5,5.5]);
ylim(ax,[-0.5,6.5]);
xlabel(ax,'x Estimate (m)');ylabel(ax,'y Estimate (m)');

end