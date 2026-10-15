function scan_plot_app(part_h_processed_scan_history,raw_scan)
%SCAN_PLOT_APP This generates an interactive visualization showing the
%raw LIDAR scan at a certain point in time, as well as the segmented,
%pre-fit-pruned, fit, and post-fit-pruned fitted wall data at the same
%time.
%
% The time of plotting can be changed using slider/text-box inputs at the 
% top of the screen.

%% UI Setup
% Create UI Figure Window
fig = uifigure;
fig.Name = "Scan Fitting Animation";

% App Layout
gl = uigridlayout(fig,[2, 4]);
gl.RowHeight = {40, '1x'};
gl.ColumnWidth = {'fit', 60, '1x', '1x'};

% Create and Position UI Components
lbl = uilabel(gl);
txt = uitextarea(gl);
sld = uislider(gl);
ax = polaraxes(gl);
ax_2 = polaraxes(gl);

%- Position Label
lbl.Layout.Row = 1;
lbl.Layout.Column = 1;

%- Position Text Field
txt.Layout.Row = 1;
txt.Layout.Column = 2;

%- Position Slider
sld.Layout.Row = 1;
sld.Layout.Column = [3 4];

%- Position Axes
ax.Layout.Row = 2;
ax.Layout.Column = [1 3];

%- Position Axes 2
ax_2.Layout.Row = 2;
ax_2.Layout.Column = 4;

%% Unpack Estimate History
time = part_h_processed_scan_history.time;
content = part_h_processed_scan_history.content;

%% Configure UI Component Appearance
lbl.Text = "Time:";

txt.Value = num2str(0,'%.2f');

sld.Limits = [time(1), time(end)];
sld.Value = time(1);

%% Initialize Plot
part_h_fitted_scan_plotter(ax,content{1});
plot_raw_scan(ax_2,raw_scan,1);

%% Assign Callback Functions
sld.ValueChangedFcn = {@SCAN_changed_time_via_slider,...
    ax,ax_2,txt,time,content,raw_scan};
sld.ValueChangingFcn = {@SCAN_changing_time_via_slider,...
    ax,ax_2,txt,time,content,raw_scan};
txt.ValueChangedFcn = {@SCAN_changed_time_via_text,...
    ax,ax_2,sld,time,content,raw_scan};

end

function SCAN_changed_time_via_text(src,event,ax,ax_2,sld,time,content,raw_scan)

% Retrieve Time and Round to Nearest Actual Value
raw_time = str2double(event.Value);
[plot_time,plot_time_loc] = find_closest_element(raw_time, time);

% Replot Walls
cla(ax);
part_h_fitted_scan_plotter(ax,content{plot_time_loc});

% Replot Raw Scan
cla(ax_2);
plot_raw_scan(ax_2,raw_scan,plot_time_loc);

% Correct Number on Slider to be on Time Axis
sld.Value = plot_time;
% Correct Number on Text to be on Time Axis
src.Value = num2str(plot_time,'%.2f');

end

function SCAN_changed_time_via_slider(src,event,ax,ax_2,txt,time,content,raw_scan)

% Retrieve Time and Round to Nearest Actual Value
raw_time = event.Value;
[plot_time,plot_time_loc] = find_closest_element(raw_time, time);

% Replot Walls
cla(ax);
part_h_fitted_scan_plotter(ax,content{plot_time_loc});

% Replot Raw Scan
cla(ax_2);
plot_raw_scan(ax_2,raw_scan,plot_time_loc);

% Correct Number on Slider to be on Time Axis
src.Value = plot_time;
% Correct Number on Text to be on Time Axis
txt.Value = num2str(plot_time,'%.2f');

end


function SCAN_changing_time_via_slider(src,event,ax,ax_2,txt,time,content,raw_scan)

% Retrieve Time and Round to Nearest Actual Value
raw_time = event.Value;
[plot_time,plot_time_loc] = find_closest_element(raw_time, time);

% Replot Walls
cla(ax);
part_h_fitted_scan_plotter(ax,content{plot_time_loc});

% Replot Raw Scan
cla(ax_2);
plot_raw_scan(ax_2,raw_scan,plot_time_loc);

% Correct Number on Text to be on Time Axis
txt.Value = num2str(plot_time,'%.2f');

end
