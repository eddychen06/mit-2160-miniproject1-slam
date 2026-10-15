function [innovation_info, fitted_labeled_scan] = ...
    part_j_innovation_calculator(scan, state_estimate)
%PART_J_INNOVATION_CALCULATOR Performs the parameter fitting for a scan,
%labels it, and builds the innovation and innovation covariance
%corresponding to the scan, relative to the current state.

%% Deconstructing the Struct
state = state_estimate.state;
state_covariance = state_estimate.covariance;

%% Fit Parameters to the Scan
fitted_labeled_scan = part_h_full_scan_fitter(scan);

%% Labeling the Fitted Scans
fitted_labeled_scan.labels = ...
    part_i_fitted_scan_labeler(fitted_labeled_scan,state_estimate);

%% Calculate the Predicted Walls Observation
predicted_walls_observation = ...
    part_f_observation_builder(state,fitted_labeled_scan.labels);

H_mat = predicted_walls_observation.matrix;
y_hat = predicted_walls_observation.vector;

%% Constructing the Measured Walls Observation
partition_count = fitted_labeled_scan.partition_count;
param_fits = fitted_labeled_scan.fits;

y_vec = NaN(2*partition_count,1);
R_mat = zeros(2*partition_count);

for it = 1:partition_count
    inds = (2*it-1):(2*it);
    y_vec(inds) = param_fits{it}.meas;
    R_mat(inds,inds) = param_fits{it}.covariance;
end

%% Calculate the Innovation
innovation = innovation_subtract(y_vec,y_hat);

%% Calculate the Innovation Covariance
S_mat = H_mat * state_covariance * H_mat' + R_mat;

%% Packing the Struct
innovation_info.innovation = innovation;
innovation_info.innovation_covariance = S_mat;
innovation_info.observation_matrix = H_mat;

end

% Applies special subtraction rule to subtract absolute angles
function innovation = ...
    innovation_subtract(measurement,predicted_measurement)

item_count = length(measurement);

innovation = NaN(item_count,1);
for it = 1:item_count
    if mod(it,2)==1 %r
        innovation(it) = measurement(it) - predicted_measurement(it);
    else %alpha
        innovation(it) = angle_subtract(measurement(it), predicted_measurement(it));
    end
end

end