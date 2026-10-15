function draw_walls_on_axes(ax,content_timepoint,...
    r_std_range,alpha_std_range)
%DRAW_WALLS_ON_AXES Given a set of axes ax and a content timepoint 
% containing the current estimate of wall parameters, and the covariances
% of those parameter estimates, this function draws all the walls on top of
% the axes.
% 
% The thickness of each wall corresponds to the standard
% deviation of the wall anglular position estimate (thinner corresponds to 
% less certain, or a larger standard deviation, and vice versa), and the 
% color of each wall corresponds to the standard deeviation of the wall
% radial position estimate (lighter corresponds to less certain, or a
% larger standard deviation, and vice versa).
%
% The inputs r_std_range and alpha_std_range determine the bounds of the
% range of standard deviations for each quantity that corresponds to the
% extremes in color or thickness.
% 
% Because r std tends not to drop that much with observations (whereas
% alpha std does), the range of light blue to dark blue is reserved for the
% nonzero range of r std, and black is reserved for true zero r std (i.e.,
% the coordinate definitions).

%% Calculate Wall Endpoints
segment_endpoint_list = ...
    wall_line_segments_from_extended_state(content_timepoint.state);

%% Extract Std Information
std_list = sqrt(diag(content_timepoint.covariance));
r_std_list = std_list(3+2*(0:9)+1);
alpha_std_list = std_list(3+2*(0:9)+2);

%% Calculate Wall Color and Thicknesses
wall_colors = color_assign_from_r_std(r_std_list,r_std_range);
wall_thicks = ...
    thickness_assign_from_alpha_std(alpha_std_list,alpha_std_range);

%% Draw Walls
for wall_id = 1:10
    plot(ax,segment_endpoint_list(wall_id,[1,3]),...
        segment_endpoint_list(wall_id,[2,4]),'-',...
        'Color',wall_colors(wall_id,:),'LineWidth',wall_thicks(wall_id));
    hold(ax,'on');
end

end

function color_list = color_assign_from_r_std(r_std_list,r_std_range)

% Color Definitions
color_high = [0.5,1,1];
color_low = 0.6*[0,0,1];
color_zero = [0,0,0];

% Detect if Zero
zero_detect = r_std_list==0;

% Calculate Std Scores
std_score = (r_std_list-r_std_range(1))./(r_std_range(2)-r_std_range(1));

% Assign Colors
color_list = ...
    (1-zero_detect).*(std_score*color_high + (1-std_score)*color_low) + ...
    zero_detect*color_zero;

end

function thickness_list = ...
    thickness_assign_from_alpha_std(alpha_std_list,alpha_std_range)

% Thickness Definitions
thick_high = 0.5;
thick_low = 5.0;

% Calculate Std Scores
std_score = (alpha_std_list-alpha_std_range(1))./...
    (alpha_std_range(2)-alpha_std_range(1));

% Assign Thicknesses
thickness_list = std_score*thick_high + (1-std_score)*thick_low;

end