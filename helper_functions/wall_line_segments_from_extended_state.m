function segment_endpoint_list = ...
    wall_line_segments_from_extended_state(state)
%WALL_LINE_SEGMENTS_FROM_EXTENDED_STATE Given an extended state estimate
%from SLAM, this calculates the intersection points between walls that are
%known to intersect.  These intersection points are used to construct the
%segment endpoints for drawing each wall as a finite line segment, not an
%infinite line.

%% Extract Wall Parameters
wall_params = NaN(10,2);
for wall_id = 1:10
    wall_params(wall_id,1) = state(3+2*(wall_id-1)+1);
    wall_params(wall_id,2) = state(3+2*(wall_id-1)+2);
end

%% Find Relevant Intersection Points
% List of Points by Wall:
% P1: W6-W1, P2: W1-W2, P3: W2-W3, P4: W3-W4, P5: W4-W5, P6: W5-W6
% P7: W10-W7, P8: W7-W8, P9: W8-W9, P10: W9-10
% List of Walls by Point:
% W1: P1-P2, W2: P2-P3, W3: P3-P4, W4: P4-P5, W5: P5-P6, W6: P6-P1
% W7: P7-P8, W8: P8-P9, W9: P9-P10, W10: P10-P7

wall_a = [6==(1:10);1==(1:10);2==(1:10);3==(1:10);4==(1:10);5==(1:10);...
    10==(1:10);7==(1:10);8==(1:10);9==(1:10)]*wall_params;
wall_b = wall_params;

r_a = wall_a(:,1);
alpha_a = wall_a(:,2);
r_b = wall_b(:,1);
alpha_b = wall_b(:,2);

pts_x = csc(alpha_a-alpha_b).*(r_b.*sin(alpha_a)-r_a.*sin(alpha_b));
pts_y = csc(alpha_a-alpha_b).*(-r_b.*cos(alpha_a)+r_a.*cos(alpha_b));

pts = [pts_x, pts_y];

pts_a = pts;
pts_b = [2==(1:10);3==(1:10);4==(1:10);5==(1:10);6==(1:10);1==(1:10);...
    8==(1:10);9==(1:10);10==(1:10);7==(1:10)]*pts;

segment_endpoint_list = [pts_a, pts_b];

end

