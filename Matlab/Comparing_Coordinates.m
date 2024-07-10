% Radius of the Earth in meters
R = 6378137;

origin_lat = 35.770764;
origin_lon = -78.674802;

points = readtable("C:\Users\malin\Downloads\Project_Oval_Test5\combined_data_test5.csv");

figure(1);

% Subplot for RTK and GPS positions
title('RTK and GPS Positions');
xlabel('X (meters)');
ylabel('Y (meters)');
hold on;

numRows = height(points);

% Arrays to store points for path plotting
rtk_path = [];
gps_path = [];
ml_path = [];

for i = 1:numRows
    fprintf("Next Point\n");
    rtk_lat = points.swift_latitude(i);
    rtk_lon = points.swift_longitude(i);
    gps_lat = points.gps_latitude(i);
    gps_lon = points.gps_longitude(i);
    ml_lat = points.ml_latitude(i);
    ml_lon = points.ml_longitude(i);

    if rtk_lat ~= 0
        rtk_point = latlon_to_xy(rtk_lat, rtk_lon, origin_lat, origin_lon, R);
        gps_point = latlon_to_xy(gps_lat, gps_lon, origin_lat, origin_lon, R);
        ml_point = latlon_to_xy(ml_lat, ml_lon, origin_lat, origin_lon, R);

        % Append points to paths
        rtk_path = [rtk_path; rtk_point];
        gps_path = [gps_path; gps_point];
        ml_path = [ml_path; ml_point];

        % Plot the paths up to the current point
        cla; % Clear the current axes
        plot(rtk_path(:, 1), rtk_path(:, 2), 'r-', 'LineWidth', 1);
        plot(gps_path(:, 1), gps_path(:, 2), 'b-', 'LineWidth', 1);
        plot(ml_path(:, 1), ml_path(:, 2), 'g-', 'LineWidth', 1);

        % Plot the most recent points with distinct markers
        plot(rtk_path(end, 1), rtk_path(end, 2), 'ro', 'MarkerSize', 6, 'MarkerFaceColor', 'r');
        plot(gps_path(end, 1), gps_path(end, 2), 'bo', 'MarkerSize', 6, 'MarkerFaceColor', 'b');
        plot(ml_path(end, 1), ml_path(end, 2), 'go', 'MarkerSize', 6, 'MarkerFaceColor', 'g');

        pause(0.1);
    end
end

function point = latlon_to_xy(lat, lon, origin_lat, origin_lon, R)
    x = haversine(origin_lat, lon, origin_lat, origin_lon, R);
    y = haversine(lat, origin_lon, origin_lat, origin_lon, R);

    if lon < origin_lon
        x = -x;
    end
    if lat < origin_lat
        y = -y;
    end

    point = [y, x];
end

function distance = haversine(lat, lon, origin_lat, origin_lon, R)
    d_lat = deg2rad(lat - origin_lat);
    d_lon = deg2rad(lon - origin_lon);

    a = sin(d_lat / 2)^2 + cos(deg2rad(origin_lat)) * cos(deg2rad(lat)) * sin(d_lon / 2)^2;
    c = 2 * atan2(sqrt(a), sqrt(1 - a));

    distance = R * c;
end
