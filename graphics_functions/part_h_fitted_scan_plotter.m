function part_h_fitted_scan_plotter(pax,fitted_scan)
%PART_H_FITTED_SCAN_PLOTTER Takes a full sweep of fitted scan data, and
%plots the raw data of the scan, categorized by partition, as well as the
%fit lines themselves. If the fitted scan data has labels attached to it,
%then the wall labels are plotted as a legend on the side.

%% Unpacking the Struct
time = fitted_scan.time;
partitions = fitted_scan.partitions;
partition_count = fitted_scan.partition_count;
fits = fitted_scan.fits;

if isfield(fitted_scan,'labels')
    has_labels = true;
    labels = fitted_scan.labels;
elseif isfield(fitted_scan,'shortcut_labels')
    has_labels = true;
    labels = fitted_scan.shortcut_labels;
else
    has_labels = false;
end

%% Plotting Colors
plotting_colors = ["#0072BD","#D95319","#EDB120","#7E2F8E","#77AC30",...
    "#4DBEEE",	"#A2142F"];
color_ind = @(ind) plotting_colors(mod(ind-1,7)+1);

%% Function for Wall Plotting
rho_wall = @(theta,r,alpha) r./cos(theta-alpha);

%% Plotting the Walls
line_width = 1.0;

for it = 1:partition_count
    min_angle = min(partitions{it}.angles);
    max_angle = max(partitions{it}.angles);
    angles_to_plot = linspace(min_angle,max_angle,101);
    polarplot(pax,angles_to_plot,rho_wall(angles_to_plot,...
        fits{it}.r,fits{it}.alpha),'-',...
        'Color',color_ind(it),'LineWidth',line_width);
    if it==1
        hold(pax,'on');
    end
end

%% Plotting the Particles
particle_size = 6;

for it = 1:partition_count
    polarplot(pax,partitions{it}.angles,partitions{it}.dists,'.',...
        'Color',color_ind(it),'MarkerSize',particle_size);
end

%% Plotting the Legend (if labels exist)
if has_labels
    legend_cells = cell(1,partition_count);
    for it = 1:partition_count
        legend_cells{it} = num2str(labels(it));
    end
    legend(pax,legend_cells);
end

%% Finishing Touches
title(pax,['Fitted Scan at Time t = ' num2str(time,'%.1f') 's']);
pax.ThetaAxisUnits = 'radians';
pax.ThetaZeroLocation = 'top';
pax.ThetaLim = [-pi,pi];
pax.RLim = [0, 3];%m

end