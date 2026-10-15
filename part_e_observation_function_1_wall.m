
function predicted_wall_observation = ...
    part_e_observation_function_1_wall(robot_state,wall_state)
%PART_E_OBSERVATION_FUNCTION_1_WALL Uses the current robot pose estimate
%and the estimate of map-frame wall parameters to calculate the
%corresponding prediction of robot-frame wall parameters (and associated
%Jacobians).

%% Checking Input Dimensionality
if ~isequal(length(robot_state),3)
    error('robot_state must be a 3D vector.');
end

if ~isequal(length(wall_state),2)
    error('wall_state must be a 2D vector.');
end

%% Unpacking the Vectors
x_R = robot_state(1);
y_R = robot_state(2);
th_R = robot_state(3);

r_j_map = wall_state(1);
alpha_j_map = wall_state(2);

%% Calculation of h

r_j_R_uncorrected = r_j_map - x_R*cos(alpha_j_map)-y_R*sin(alpha_j_map);
alpha_j_R_uncorrected = alpha_j_map - th_R;

is_r_correct = (r_j_R_uncorrected >= 0);

if is_r_correct
    r_j_R = r_j_R_uncorrected;
    alpha_j_R = alpha_j_R_uncorrected;
else
    r_j_R = -r_j_R_uncorrected;
    alpha_j_R = alpha_j_R_uncorrected - pi;
end

h_j_R = [r_j_R;alpha_j_R];

%% Calculation of H sub-matrices
if is_r_correct
    H_j_R = [
        -cos(alpha_j_map), -sin(alpha_j_map), 0;
        0, 0, -1;
    ];
    H_j_map_j = [
        1, x_R*sin(alpha_j_map) - y_R*cos(alpha_j_map);
        0, 1;
    ];
else
    H_j_R = [
        cos(alpha_j_map), sin(alpha_j_map), 0;
        0, 0, -1;
    ];
    H_j_map_j = [
        -1, -x_R*sin(alpha_j_map) + y_R*cos(alpha_j_map);
        0, 1;
    ];
end

%% Packing the Struct
predicted_wall_observation.observation = h_j_R;
predicted_wall_observation.r = r_j_R;
predicted_wall_observation.alpha = alpha_j_R;
predicted_wall_observation.H_R = H_j_R;
predicted_wall_observation.H_map = H_j_map_j;

end