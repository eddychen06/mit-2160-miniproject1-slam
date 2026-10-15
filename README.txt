Explanation of Data Files (data\ folder)

odom.csv
    Describes wheel motion of robot wheels (in terms of distance traveled,
        not wheel rotation angle).
        Time: Time in seconds. Synchronized with scan data Time.
            Sample rate of 100 Hz.
        s_right: Right wheel traveled path length (in meters)
        s_left: Left wheel traveled path length (in meters)

scan_dist.csv
    Describes distance scan sensed by LIDAR sensor
        First column is Time in seconds. Synchronized with odom data Time.
            Sample rate of 2 Hz.
        First row is Scan Angle (in radians). Negative angles are to the
            right of forward, and positive angles are to the left of
            forward (as per the right-handed standard).
        Each row after the first corresponds to the distances (in meters)
            detected at each scan angle, at a given time. The LIDAR sensor
            has an operating range of 0.3 m - 3 m. A reported distance of
            -1 corresponds to no walls in range.

scan_partition.csv
    Describes a partitioning of the detected distance data into different
        distinct walls
        First column and first row are same as scan_dist.csv
        Each row after the first corresponds to a partitioning of the
            distance data into different walls, for a given time.  Values
            of -1 correspond to no wall being detected.  Multiple instances
            of the same partition value within the same time correspond to
            their angles' distance data belonging to the same wall.  The
            partition values do not necessarily correspond to the same
            walls across time steps; instead, they simply distinguish walls
            within a single time step.  Specifically, they count up from 1
            to the number of walls in view, for any given time step.

scan_partition_labels.csv
    Details a mapping between the partition values for each time step and
        the actual walls they encode
        First column is same as scan_dist.csv
        First row corresponds to a partition value in scan_partition.csv
        Each row after the first provides the actual wall labels
            corresponding to each partition value at each time step.


Explanation of functions and scripts

Functions in the helper_functions\ and graphics_functions\ folder are meant
    to be used without modification by you.  Feel free to modify them, but
    you shouldn't have to do so, even to get full extra credit.
Functions in the root folder, whose names all start with "part_",
    require modification by you in order to function (with the
    exception of part_i_fitted_scan_labeler, which can be used
    unmodified for regular credit).  Specifically, you will need to
    replace any instances of "???" with code.  You may also want to add
    additional lines of code, leading up to the equations with "???".
    You are free to edit other existing lines of code in the functions,
    but you should not need to, in order to get full extra credit.
Scripts in the root folder, whose names all start with "example_",
    show examples of how to use the functions.  You are free to reuse
    code in the example scripts, though you will have to write
    additional code to receive full credit.  The figures plotted in
    the example scripts are not intended to be examples of best practices
    in plot formatting; their purpose is simply to demonstrate examples how
    to extract certain trajectories from the result structs.

    The scripts are intended to be run in the following order; they are not
    independent:
    1. example_data_loader.m
    2. example_1.m
    3. example_2.m