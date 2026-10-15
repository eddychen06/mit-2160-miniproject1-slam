function struct_out = data_interleaver(odom_struct,scan_struct)
%DATA_INTERLEAVER Combines the odometry data and scan data in their time
%series.  Whenever odometry data and scan data occur at the same time,
%odometry data is placed first.

% Combine raw structures
struct_out.odom = odom_struct.odom;
struct_out.raw_scan = scan_struct.raw_scan;

% Preparations for the interleaving operation
struct_out.general.time_count = struct_out.odom.time_count +...
    struct_out.raw_scan.time_count;
struct_out.general.time = NaN(struct_out.general.time_count,1);
struct_out.general.content =...
    cell(struct_out.general.time_count,1);
struct_out.general.is_scan = false(struct_out.general.time_count,1);

odom_time = struct_out.odom.time;
scan_time = struct_out.raw_scan.time;

time_final_odom = odom_time(end);
time_final_scan = scan_time(end);

odom_is_complete = false;
scan_is_complete = false;

odom_index = 1;
scan_index = 1;
master_index = 1;

%last_time = -1.0;
next_odom_time = struct_out.odom.time(1);
next_scan_time = struct_out.raw_scan.time(1);

while ~(odom_is_complete && scan_is_complete)
    if ~odom_is_complete
        if (odom_time(odom_index) <= next_scan_time)||scan_is_complete
            struct_out.general.content{master_index} =...
                odom_struct.general.content{odom_index};
            struct_out.general.time(master_index) = ...
                odom_struct.general.content{odom_index}.time;

            if odom_time(odom_index) >= time_final_odom
                odom_is_complete = true;
            else
                odom_index = odom_index + 1;
                next_odom_time = odom_time(odom_index);
            end
            master_index = master_index + 1;
        end
    end

    if ~scan_is_complete
        % The '<' ensures that scans always happen after odom updates
        % whenever they happen at the same time
        if (scan_time(scan_index) < next_odom_time)||odom_is_complete
            struct_out.general.content{master_index} =...
                scan_struct.general.content{scan_index};
            struct_out.general.is_scan(master_index) = true;
            struct_out.general.time(master_index) = ...
                scan_struct.general.content{scan_index}.time;

            if scan_time(scan_index) >= time_final_scan
                scan_is_complete = true;
            else
                scan_index = scan_index + 1;
                next_scan_time = scan_time(scan_index);
            end
            master_index = master_index + 1;
        end
    end
end

end