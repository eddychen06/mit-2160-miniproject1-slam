function plot_raw_scan(pax,raw_scan,time_index)
%PLOT_RAW_SCAN Takes a full sweep of raw scan data, and
%plots the raw data of the scan onto polar axes, unmodified.

%% Unpacking the Struct
time = raw_scan.time(time_index);

%% Plotting the Particles
particle_size = 6;

angles_all = raw_scan.angles;
dists_all = raw_scan.dists(time_index,:);

keep_idxs = dists_all > 0;

polarplot(pax,angles_all(keep_idxs),dists_all(keep_idxs),'.',...
    'Color','k','MarkerSize',particle_size);

%% Finishing Touches
title(pax,['Raw Scan at Time t = ' num2str(time,'%.1f') 's']);
pax.ThetaAxisUnits = 'radians';
pax.ThetaZeroLocation = 'top';
pax.ThetaLim = [-pi,pi];
pax.RLim = [0, 3];%m

end

