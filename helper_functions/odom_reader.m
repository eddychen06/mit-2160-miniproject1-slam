function struct_out = odom_reader(odom_file)
%ODOM_READER Reads the odometry file to create odometry events in a cell
%array, indexed by time

% Import csv as raw matrix (ignores first label row)
M = readmatrix(odom_file);

% Extract core vectors from raw matrix
struct_out.odom.time = M(:,1);
struct_out.odom.s_right = M(:,2);
struct_out.odom.s_left = M(:,3);
struct_out.odom.time_count = length(struct_out.odom.time);

% Generate general time structure
struct_out.general.time_count = struct_out.odom.time_count;
struct_out.general.time = struct_out.odom.time;
struct_out.general.content = cell(struct_out.general.time_count,1);
for time_iter = 1:struct_out.general.time_count
    struct_out.general.content{time_iter}.type = 'odom';
    struct_out.general.content{time_iter}.time = ...
        struct_out.general.time(time_iter);
    struct_out.general.content{time_iter}.s_right = ...
        struct_out.odom.s_right(time_iter);
    struct_out.general.content{time_iter}.s_left = ...
        struct_out.odom.s_left(time_iter);
end

end