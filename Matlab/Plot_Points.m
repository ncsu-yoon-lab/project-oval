% Radius of the Earth in meters
R = 6378137;

origin_lat = 35.770764;
origin_lon = -78.674802;

points = readtable("C:\Users\malin\Documents\GitHub\project-oval\ML\position_data_trainer_combined.csv");

figure;

% Create plot
title('RTK and GPS Positions');
xlabel('X (meters)');
ylabel('Y (meters)');
hold on;

% Extract data
rtk_lat = points.swift_latitude_filtered;
rtk_lon = points.swift_longitude_filtered;
gps_lat = points.gps_latitude_filtered;
gps_lon = points.gps_longitude_filtered;

% Convert latitudes and longitudes to XY coordinates
rtk_points = arrayfun(@(i) latlon_to_xy(rtk_lat(i), rtk_lon(i), origin_lat, origin_lon, R), 1:length(rtk_lat), 'UniformOutput', false);
gps_points = arrayfun(@(i) latlon_to_xy(gps_lat(i), gps_lon(i), origin_lat, origin_lon, R), 1:length(gps_lat), 'UniformOutput', false);

% Convert cell arrays to matrices
rtk_points = cell2mat(rtk_points');
gps_points = cell2mat(gps_points');

% Plot RTK and GPS points
plot(rtk_points(:, 1), rtk_points(:, 2), 'ro', 'MarkerSize', 6, 'MarkerFaceColor', 'r');
plot(gps_points(:, 1), gps_points(:, 2), 'bo', 'MarkerSize', 6, 'MarkerFaceColor', 'b');

legend('RTK', 'GPS');

% Function to convert latitude and longitude to XY coordinates
function point = latlon_to_xy(lat, lon, origin_lat, origin_lon, R)
    x = haversine(origin_lat, lon, origin_lat, origin_lon, R);
    y = haversine(lat, origin_lon, origin_lat, origin_lon, R);

    if lon < origin_lon
        x = -x;
    end
    if lat < origin_lat
        y = -y;
    end

    point = [x, y]; % Note: Adjusted x and y order to match typical plotting (X, Y)
end

% Haversine function to calculate distance
function distance = haversine(lat, lon, origin_lat, origin_lon, R)
    d_lat = deg2rad(lat - origin_lat);
    d_lon = deg2rad(lon - origin_lon);

    a = sin(d_lat / 2)^2 + cos(deg2rad(origin_lat)) * cos(deg2rad(lat)) * sin(d_lon / 2)^2;
    c = 2 * atan2(sqrt(a), sqrt(1 - a));

    distance = R * c;
end
