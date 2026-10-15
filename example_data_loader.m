% The core code for producing the data structs used to feed the functions
% of the Kalman filter and of wall fitting.  Except for the option in
% shortcut labeling, should be implemented without modification.

addpath('data\');
addpath('helper_functions\');
addpath('graphics_functions\');

odom_struct = odom_reader('data\odom.csv');

% If using the shortcut labeling, include this:
scan_struct = scan_reader('data\scan_dist.csv',...
    'data\scan_partition.csv','data\scan_partition_labels.csv');
% If making your own labeling, include this:
%scan_struct = scan_reader('data\scan_dist.csv','data\scan_partition.csv');

data_struct = data_interleaver(odom_struct,scan_struct);