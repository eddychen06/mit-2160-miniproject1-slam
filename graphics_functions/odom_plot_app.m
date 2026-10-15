function odom_plot_app(part_b_estimate_history)
%ODOM_PLOT_APP This generates an interactive visualization showing the
%odometry-estimated trajectory of the robot up to a certain point in time,
%as well as the robot's current pose estimate at that time, and the
% (x,y) covariance of that estimate at that time.  The covariance is
% illustrated using a Z-score isoline of the Gaussian distribution.
%
% The time of plotting and the Z-score for covariance plotting can both be
% changed using slider/text-box inputs at the top of the screen.

%% UI Setup
% Create UI Figure Window
fig = uifigure;
fig.Name = "Odometry Animation";

% App Layout
gl = uigridlayout(fig,[3, 3]);
gl.RowHeight = {40, 40, '1x'};
gl.ColumnWidth = {'fit', 60, '1x'};

% Create and Position UI Components
lbl_time = uilabel(gl);
txt_time = uitextarea(gl);
sld_time = uislider(gl);

lbl_z = uilabel(gl);
txt_z = uitextarea(gl);
sld_z = uislider(gl);

ax = uiaxes(gl);

%- Position Time Label
lbl_time.Layout.Row = 1;
lbl_time.Layout.Column = 1;

%- Position Time Text Field
txt_time.Layout.Row = 1;
txt_time.Layout.Column = 2;

%- Position Time Slider
sld_time.Layout.Row = 1;
sld_time.Layout.Column = 3;

%- Position Z-Score Label
lbl_z.Layout.Row = 2;
lbl_z.Layout.Column = 1;

%- Position Z-Score Text Field
txt_z.Layout.Row = 2;
txt_z.Layout.Column = 2;

%- Position Z-Score Slider
sld_z.Layout.Row = 2;
sld_z.Layout.Column = 3;

%- Position Axes
ax.Layout.Row = 3;
ax.Layout.Column = [1 3];

%% Unpack Estimate History
time = part_b_estimate_history.time;
content = part_b_estimate_history.content;
x = NaN(part_b_estimate_history.time_count,1);
y = NaN(part_b_estimate_history.time_count,1);
th = NaN(part_b_estimate_history.time_count,1);
for time_iter = 1:part_b_estimate_history.time_count
    x(time_iter) = ...
        content{time_iter}.state(1);
    y(time_iter) = ...
        content{time_iter}.state(2);
    th(time_iter) = ...
        content{time_iter}.state(3);
end

%% Configure UI Component Appearance
lbl_time.Text = "Time:";

txt_time.Value = num2str(0,'%.2f');

sld_time.Limits = [time(1), time(end)];
sld_time.Value = time(1);

lbl_z.Text = "Z-Score:";

txt_z.Value = num2str(0.2,'%.2f');

sld_z.Limits = [0.1, 5];
sld_z.Value = 0.2;

%% Initialize Plot
ODOM_replot_up_to_time(ax,time,x,y,th,1);

x_robot = x(1);
y_robot = y(1);
th_robot = th(1);

draw_robot_on_axes(ax,x_robot,y_robot,th_robot);

hold(ax,'on');
plot_2d_covariance_matrix_on_axes(ax,content{1}.covariance(1:2,1:2),...
    [x_robot; y_robot],sld_z.Value,0.5,'r');

%% Assign Callback Functions
sld_time.ValueChangedFcn = {@ODOM_changed_time_via_slider,...
    ax,txt_time,sld_z,time,x,y,th,content};
sld_time.ValueChangingFcn = {@ODOM_changing_time_via_slider,...
    ax,txt_time,sld_z,time,x,y,th,content};
txt_time.ValueChangedFcn = {@ODOM_changed_time_via_text,...
    ax,sld_time,sld_z,time,x,y,th,content};
sld_z.ValueChangedFcn = {@ODOM_changed_z_via_slider,...
    ax,sld_time,txt_z,time,x,y,th,content};
sld_z.ValueChangingFcn = {@ODOM_changing_z_via_slider,...
    ax,sld_time,txt_z,time,x,y,th,content};
txt_z.ValueChangedFcn = {@ODOM_changed_z_via_text,...
    ax,sld_time,sld_z,time,x,y,th,content};

end

function ODOM_changed_time_via_text(src,event,ax,sld_time,sld_z,time,x,y,th,content)

% Retrieve Time and Round to Nearest Actual Value
raw_time = str2double(event.Value);
[plot_time,plot_time_loc] = find_closest_element(raw_time, time);

% Retrieve Robot Position
x_robot = x(plot_time_loc);
y_robot = y(plot_time_loc);
th_robot = th(plot_time_loc);

% Replot Trajectory
ODOM_replot_up_to_time(ax,time,x,y,th,plot_time_loc);

% Draw Robot
draw_robot_on_axes(ax,x_robot,y_robot,th_robot);

% Plot Covariance Matrix
hold(ax,'on');
plot_2d_covariance_matrix_on_axes(ax,content{plot_time_loc}.covariance(1:2,1:2),...
    [x_robot; y_robot],sld_z.Value,0.5,'r');

% Correct Number on Slider to be on Time Axis
sld_time.Value = plot_time;
% Correct Number on Text to be on Time Axis
src.Value = num2str(plot_time,'%.2f');

end

function ODOM_changed_time_via_slider(src,event,ax,txt_time,sld_z,time,x,y,th,content)

% Retrieve Time and Round to Nearest Actual Value
raw_time = event.Value;
[plot_time,plot_time_loc] = find_closest_element(raw_time, time);

% Retrieve Robot Position
x_robot = x(plot_time_loc);
y_robot = y(plot_time_loc);
th_robot = th(plot_time_loc);

% Replot Trajectory
ODOM_replot_up_to_time(ax,time,x,y,th,plot_time_loc);

% Draw Robot
draw_robot_on_axes(ax,x_robot,y_robot,th_robot);

% Plot Covariance Matrix
hold(ax,'on');
plot_2d_covariance_matrix_on_axes(ax,content{plot_time_loc}.covariance(1:2,1:2),...
    [x_robot; y_robot],sld_z.Value,0.5,'r');

% Correct Number on Slider to be on Time Axis
src.Value = plot_time;
% Correct Number on Text to be on Time Axis
txt_time.Value = num2str(plot_time,'%.2f');

end


function ODOM_changing_time_via_slider(src,event,ax,txt_time,sld_z,time,x,y,th,content)

% Retrieve Time and Round to Nearest Actual Value
raw_time = event.Value;
[plot_time,plot_time_loc] = find_closest_element(raw_time, time);

% Retrieve Robot Position
x_robot = x(plot_time_loc);
y_robot = y(plot_time_loc);
th_robot = th(plot_time_loc);

% Replot Trajectory
ODOM_replot_up_to_time(ax,time,x,y,th,plot_time_loc);

% Draw Robot
draw_robot_on_axes(ax,x_robot,y_robot,th_robot);

% Plot Covariance Matrix
hold(ax,'on');
plot_2d_covariance_matrix_on_axes(ax,content{plot_time_loc}.covariance(1:2,1:2),...
    [x_robot; y_robot],sld_z.Value,0.5,'r');

% Correct Number on Text to be on Time Axis
txt_time.Value = num2str(plot_time,'%.2f');

end


function ODOM_replot_up_to_time(ax,time,x,y,th,time_loc)

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


function ODOM_changed_z_via_text(src,event,ax,sld_time,sld_z,time,x,y,th,content)

% Retrieve Time and Round to Nearest Actual Value
raw_time = sld_time.Value;
[plot_time,plot_time_loc] = find_closest_element(raw_time, time);

% Retrieve Robot Position
x_robot = x(plot_time_loc);
y_robot = y(plot_time_loc);
th_robot = th(plot_time_loc);

% Replot Trajectory
ODOM_replot_up_to_time(ax,time,x,y,th,plot_time_loc);

% Draw Robot
draw_robot_on_axes(ax,x_robot,y_robot,th_robot);

% Threshold z
new_z = min(max(str2double(event.Value),sld_z.Limits(1)),sld_z.Limits(2));

% Plot Covariance Matrix
hold(ax,'on');
plot_2d_covariance_matrix_on_axes(ax,content{plot_time_loc}.covariance(1:2,1:2),...
    [x_robot; y_robot],new_z,0.5,'r');

% Correct Number on Slider
sld_z.Value = new_z;
% Correct Number on Text
src.Value = num2str(new_z);

end

function ODOM_changed_z_via_slider(src,event,ax,sld_time,txt_z,time,x,y,th,content)

% Retrieve Time and Round to Nearest Actual Value
raw_time = sld_time.Value;
[plot_time,plot_time_loc] = find_closest_element(raw_time, time);

% Retrieve Robot Position
x_robot = x(plot_time_loc);
y_robot = y(plot_time_loc);
th_robot = th(plot_time_loc);

% Replot Trajectory
ODOM_replot_up_to_time(ax,time,x,y,th,plot_time_loc);

% Draw Robot
draw_robot_on_axes(ax,x_robot,y_robot,th_robot);

% Plot Covariance Matrix
hold(ax,'on');
plot_2d_covariance_matrix_on_axes(ax,content{plot_time_loc}.covariance(1:2,1:2),...
    [x_robot; y_robot],event.Value,0.5,'r');

% Correct Number on Text to be on Time Axis
txt_z.Value = num2str(event.Value);

end


function ODOM_changing_z_via_slider(src,event,ax,sld_time,txt_z,time,x,y,th,content)

% Retrieve Time and Round to Nearest Actual Value
raw_time = sld_time.Value;
[plot_time,plot_time_loc] = find_closest_element(raw_time, time);

% Retrieve Robot Position
x_robot = x(plot_time_loc);
y_robot = y(plot_time_loc);
th_robot = th(plot_time_loc);

% Replot Trajectory
ODOM_replot_up_to_time(ax,time,x,y,th,plot_time_loc);

% Draw Robot
draw_robot_on_axes(ax,x_robot,y_robot,th_robot);

% Plot Covariance Matrix
hold(ax,'on');
plot_2d_covariance_matrix_on_axes(ax,content{plot_time_loc}.covariance(1:2,1:2),...
    [x_robot; y_robot],event.Value,0.5,'r');

% Correct Number on Text to be on Time Axis
txt_z.Value = num2str(event.Value);

end