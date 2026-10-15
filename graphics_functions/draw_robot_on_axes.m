function draw_robot_on_axes(ax,x,y,th)
%DRAW_ROBOT_ON_AXES Given a set of axes ax and a robot pose x,y,th, this
%function draws a cartoon representation of the robot in the supplied pose
%on top of the axes.

%% Define Colors
base_color = 0.6*[1,1,1];
tire_color = 0.2*[1,1,1];
scanner_color = 0.3*[1,1,1];
scan_color = 0.8*[0,1,0];
scan_alpha = 0.1;

%% Define Robot Dimensions
r_len = 0.3;%m
r_wid = 0.25;%m

tire_len = 0.1;%
tire_wid = 0.05;%m

tire_len_pos = 0.125;
tire_wid_pos = 0.15;

scanner_tower_diam = 0.075;%m
scanner_radius = 0.0875;%m
scanner_ang_extent = 2*pi/3;%rad

scan_radius = 3.0;%m

%% Define HGTransform
transform_mat = makehgtform("translate",[x,y,0])*...
    makehgtform("zrotate",th);
h = hgtransform(ax,'Matrix',transform_mat);

%% Draw Robot
% Scan Region
th_samp = scanner_ang_extent*(-1:0.05:1);
scan_patch_x = [0, scan_radius*cos(th_samp), 0];
scan_patch_y = [0, scan_radius*sin(th_samp), 0];

patch(scan_patch_x,scan_patch_y,scan_color,...
    'FaceAlpha',scan_alpha,...
    'EdgeColor','none','Parent',h);

% Base
rectangle('Position',[-r_len/2,-r_wid/2,r_len,r_wid],...
    'FaceColor',base_color,'Parent',h);
% Tires
rectangle('Position',[tire_len_pos-tire_len/2,tire_wid_pos-tire_wid/2,...
    tire_len,tire_wid],...
    'FaceColor',tire_color,'Parent',h);
rectangle('Position',[tire_len_pos-tire_len/2,-tire_wid_pos-tire_wid/2,...
    tire_len,tire_wid],...
    'FaceColor',tire_color,'Parent',h);
rectangle('Position',[-tire_len_pos-tire_len/2,tire_wid_pos-tire_wid/2,...
    tire_len,tire_wid],...
    'FaceColor',tire_color,'Parent',h);
rectangle('Position',[-tire_len_pos-tire_len/2,-tire_wid_pos-tire_wid/2,...
    tire_len,tire_wid],...
    'FaceColor',tire_color,'Parent',h);

% LIDAR Scanner
scanner_patch_x = [0, scanner_radius*cos(th_samp), 0];
scanner_patch_y = [0, scanner_radius*sin(th_samp), 0];

patch(scanner_patch_x,scanner_patch_y,scanner_color,'Parent',h);

rectangle('Position',[-scanner_tower_diam/2,-scanner_tower_diam/2,...
    scanner_tower_diam,scanner_tower_diam],...
    'Curvature',[1 1],'FaceColor',scanner_color,'Parent',h);

end

