function struct_out = ...
    scan_reader(scan_file,scan_partition_file,scan_partition_label_file)
%SCAN_READER Reads the scan file, scan partition file, and scan parition
% label file to create scan events in a cell array, indexed by time. If the
% scan partition label file is left out, then the cell array is still
% properly built, but without the 'shortcut labels' included.

%% Import Scan File
% Import csv as raw matrix
M1 = readmatrix(scan_file);

% Extract core vectors from raw matrix
struct_out.raw_scan.time = M1(2:end,1);
struct_out.raw_scan.angles = M1(1,2:end);
struct_out.raw_scan.dists = M1(2:end,2:end);
struct_out.raw_scan.time_count = ...
    length(struct_out.raw_scan.time);
struct_out.raw_scan.angle_count = ...
    length(struct_out.raw_scan.angles);

%% Import Scan Partition File
% Import csv as raw matrix
M2 = readmatrix(scan_partition_file);

% Extract core vectors from raw matrix
struct_out.raw_scan.parts = M2(2:end,2:end);

%% Import Scan Partition Label (for shortcut)
if nargin > 2
    % Import csv as raw matrix
    M3 = readmatrix(scan_partition_label_file);
    % Extract core vectors from raw matrix
    struct_out.raw_scan.shortcut_labels = M3(2:end,2:end);
    struct_out.raw_scan.shortcut_label_count = ...
        max(struct_out.raw_scan.shortcut_labels,[],'all');
end

%%
% Generate general time structure
struct_out.general.time_count = struct_out.raw_scan.time_count;
struct_out.general.time = struct_out.raw_scan.time;
struct_out.general.content =...
    cell(struct_out.general.time_count,1);
for time_iter = 1:struct_out.general.time_count
    struct_out.general.content{time_iter}.type = 'scan';
    struct_out.general.content{time_iter}.time = ...
        struct_out.general.time(time_iter);
    [struct_out.general.content{time_iter}.partitions,...
        struct_out.general.content{time_iter}.partition_count] = ...
        manual_partitioner(struct_out.raw_scan.angles,...
        struct_out.raw_scan.dists(time_iter,:),...
        struct_out.raw_scan.parts(time_iter,:));

    if nargin > 2
        part_count = struct_out.general.content{time_iter}.partition_count;
        struct_out.general.content{time_iter}.shortcut_labels = ...
            struct_out.raw_scan.shortcut_labels(time_iter,1:part_count);
        for part_iter = 1:part_count
            struct_out.general.content{time_iter}...
                .partitions{part_iter}.shortcut_label =...
                struct_out.general.content{time_iter}...
                .shortcut_labels(part_iter);
        end
    end
end

end

function [partitions, partition_count] = ...
    manual_partitioner(angles,dists,parts)

% Partition Count is max of partition labels
partition_count = max(parts,[],'all');

% Create partition cell array
partitions = cell(1,partition_count);

% Fill out partition cell array with partitioned angles and distances
for part_it = 1:partition_count
    part_indices = find(parts==part_it);
    partitions{part_it}.angles = angles(part_indices);
    partitions{part_it}.dists = dists(part_indices);
    partitions{part_it}.angle_count = length(part_indices);
end

end