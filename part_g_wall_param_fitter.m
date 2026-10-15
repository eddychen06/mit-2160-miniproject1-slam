function wall_params = part_g_wall_param_fitter(wall_partition)
%PART_G_WALL_PARAM_FITTER Takes the scan data for a single wall partition
%and estimates the measured robot-frame wall parameters for that wall.
%Also calculates the fit covariance for those parameters.

%% Unpacking the Struct
angle_count = wall_partition.angle_count;
angles = wall_partition.angles;
dists = wall_partition.dists;

if angle_count < 3
    error(['If a wall has 2 or fewer points, the covariance of its fit',...
        ' cannot be well-defined.']);
end

%% Defining the Error Function for the Fit
% Remember: Use .* and ./ for element-wise multiplication
error_function = @(r,alpha,angle,dist) dist .* cos(angle-alpha)-r;

%% Setting up the Fit
% Basics of the fit
x_data = [angles;dists]';
y_data = zeros(angle_count,1);
fun = @(x,xdata) error_function(x(1),x(2),xdata(:,1),xdata(:,2));

% Initial Guess
initial_param_guess = [1,1,1,1,1;-pi/2,0,pi/2,0.99*pi,-0.99*pi];

% Bounds and Options
LB = [0,-pi]; UB = [6,pi];
options = optimset('Display','off');

%% Run the Fit Multiple Times from Different Initial Conditions
% and picking the best result (a good strategy when dealing with nonlinear
% fitting)

for run_it = 1:5
    % Running the Fit
    [param_fit,~,residual,~,~,~,jacobian] = lsqcurvefit(...
        fun,initial_param_guess(:,run_it),x_data,y_data,LB,UB,options);

    % Post-Processing the Fit
    fit_covariance = ...
        (inv(jacobian'*jacobian)*(residual'*residual))./(angle_count-2);

    % Judgment
    if run_it==1
        param_fit_chosen = param_fit;
        covar_chosen = fit_covariance;
    else
        if trace(fit_covariance)<trace(covar_chosen)
            param_fit_chosen = param_fit;
            covar_chosen = fit_covariance;
        end
    end
end

r_fit = param_fit_chosen(1);
alpha_fit = param_fit_chosen(2);

%% Packing the Struct
wall_params.meas = param_fit_chosen;
wall_params.r = r_fit;
wall_params.alpha = alpha_fit;
wall_params.covariance = covar_chosen;

end