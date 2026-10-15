function predicted_walls_observation = ...
    part_f_observation_builder(state,obs_walls_list)
%PART_F_OBSERVATION_BUILDER Uses the current full augmented state estimate
%and a list of wall ID numbers to generate the predicted set of robot-frame
%wall parameters, and the Jacobian H (the observation matrix), which
%relates the augmented state to the predicted set of observed robot-frame
%wall parameters.

wall_count = 10;
state_length = 3+2*wall_count;
obs_wall_count = length(obs_walls_list);

%% Checking Input Dimensionality and Walls List
if ~isequal(length(state),state_length)
    error(['state must be a ', num2str(state_length), '-D vector.']);
end

if ~all(and(obs_walls_list<=wall_count,obs_walls_list>=1))
    error(['Walls in walls_list must be limited from 1 to ', ...
        num2str(wall_count), '.']);
end

%% Unpacking the State Vector
robot_state = state(1:3);
walls_state = cell(1,wall_count);
for wall_it = 1:wall_count
    walls_state{wall_it} = state((3+1+2*(wall_it-1)):(3+2+2*(wall_it-1)));
end

%% Generate Predicted Observation Vector and Matrix
observation_vector = NaN(2*obs_wall_count,1);
observation_matrix = zeros(2*obs_wall_count,state_length);

for obs_wall_it = 1:obs_wall_count
    wall_id = obs_walls_list(obs_wall_it);
    obs_wall_output_indices = (2*obs_wall_it - 1):(2*obs_wall_it);
    obs_wall_input_indices = (2*wall_id + 2):(2*wall_id + 3);

    wall_obs = part_e_observation_function_1_wall(...
        robot_state,walls_state{wall_id});

    observation_vector(obs_wall_output_indices) = wall_obs.observation;
    observation_matrix(obs_wall_output_indices,1:3) = wall_obs.H_R;
    observation_matrix(obs_wall_output_indices,obs_wall_input_indices)...
        = wall_obs.H_map;
end

%% Packing the Struct
predicted_walls_observation.vector = observation_vector;
predicted_walls_observation.matrix = observation_matrix;

end