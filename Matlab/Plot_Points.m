% Radius of the Earth in meters
R = 6378137;

origin_lat = 35.770764;
origin_lon = -78.674802;

points = readtable("C:\Users\malin\Documents\GitHub\project-oval\Data\data_gps_logger_test4.csv");

figure;

% Create plot
title('RTK and GPS Positions');
xlabel('X (meters)');
ylabel('Y (meters)');
hold on;

% Extract data
rtk_lat = points.latitude;
rtk_lon = points.longitude;
gps_lat = gps_converter(points.gps_lat);
gps_lon = gps_converter(points.gps_lon);
hdop = points.hdop;

% Convert latitudes and longitudes to XY coordinates
gps_points = arrayfun(@(i) latlon_to_xy(gps_lat(i), gps_lon(i), origin_lat, origin_lon, R), 1:length(gps_lat), 'UniformOutput', false);
rtk_points = arrayfun(@(i) latlon_to_xy(rtk_lat(i), rtk_lon(i), origin_lat, origin_lon, R), 1:length(gps_lat), 'UniformOutput', false);

% Convert cell arrays to matrices
rtk_points = cell2mat(rtk_points');
gps_points = cell2mat(gps_points');

% Plot RTK points
scatter(rtk_points(:, 1), rtk_points(:, 2), 50, 'r', 'filled', 'DisplayName', 'RTK');

% Plot GPS points with sizes corresponding to HDOP
scatter(gps_points(:, 1), gps_points(:, 2), hdop * 10, 'b', 'filled', 'DisplayName', 'GPS'); % Scale HDOP for visibility

legend;

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

% Converts the gps from DDmm.mmmm to dd.dddd
function point = gps_converter(original_point)
    degrees = floor(original_point / 100);
    minutes = original_point - (degrees * 100);
    point = degrees + (minutes / 60);

    if point > 50
        point = point * -1.0;
    end
end


% Haversine function to calculate distance
function distance = haversine(lat, lon, origin_lat, origin_lon, R)
    d_lat = deg2rad(lat - origin_lat);
    d_lon = deg2rad(lon - origin_lon);

    a = sin(d_lat / 2)^2 + cos(deg2rad(origin_lat)) * cos(deg2rad(lat)) * sin(d_lon / 2)^2;
    c = 2 * atan2(sqrt(a), sqrt(1 - a));

    distance = R * c;
end
