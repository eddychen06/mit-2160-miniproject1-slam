function scan_labeling = part_i_fitted_scan_labeler(fitted_scan,...
    state_estimate)
%PART_I_FITTED_SCAN_LABELER This function takes a fitted scan and labels
%each of the fitted scan's partitions.  The state estimate is provided as a
%resource that can be used to perform this labeling.
%For extra credit, set use_shortcut_labels = false and create an algorithm
%which labels the partitions.

%% Unpacking the Structs
if isfield(fitted_scan,'shortcut_labels')
    shortcut_labels = fitted_scan.shortcut_labels;
end
fits = fitted_scan.fits;
state = state_estimate.state;
state_covar = state_estimate.covariance;

%% Labeling the Fitted Scans

use_shortcut_labels = true;

if use_shortcut_labels
    scan_labeling = shortcut_labels;
else
    % For extra credit, put your algorithm here (and replace '0').
    scan_labeling = 0;
end

end