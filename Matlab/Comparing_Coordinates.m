% Radius of the Earth in meters
R = 6378137;


origin_lat = 35.770764;
origin_lon = -78.674802;

points = readtable('C:\Users\malin\Documents\GitHub\project-oval\Data\data_gps_logger_test4.csv');

figure(1);

% Subplot for RTK and GPS positions
subplot(2,1,1);
hold on;
title('RTK and GPS Positions');
xlabel('X (meters)');
ylabel('Y (meters)');

% Subplot for percent differences
subplot(2,1,2);
hold on;
title('Percent Differences');
xlabel('Point Index');
ylabel('Percent Difference (%)');

numRows = height(points);

for i = 1:numRows
    fprintf("Next Point\n");
    rtk_lat = points.latitude(i);
    rtk_lon = points.longitude(i);
    gps_lat = convert_degrees(points.gps_lat(i));
    gps_lon = -1.0 * convert_degrees(points.gps_lon(i));

    if rtk_lat ~= 0

        rtk_point = latlon_to_xy(rtk_lat, rtk_lon, origin_lat, origin_lon, R);
    
        gps_point = latlon_to_xy(gps_lat, gps_lon, origin_lat, origin_lon, R);

        % Plot RTK and GPS points
        subplot(2,1,1);
        plot(rtk_point(1), rtk_point(2), 'ro', 'MarkerSize', 3, 'MarkerFaceColor', 'r');
        plot(gps_point(1), gps_point(2), 'bo', 'MarkerSize', 3, 'MarkerFaceColor', 'b');

        % Calculate percent difference
        percent_diff_lat = abs(abs(rtk_lat - gps_lat) / (rtk_lat) * 100);
        percent_diff_lon = abs(abs(rtk_lon - gps_lon) / (rtk_lon) * 100);

        % Plot percent difference
        subplot(2,1,2);
        plot(i, percent_diff_lat, 'r.');
        plot(i, percent_diff_lon, 'b.');
        legend('Latitude', 'Longitude');
        drawnow;
        
    
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

function decimalDegrees = convert_degrees(point)
    degrees = floor(point / 100);

    minutes = point - degrees * 100;

    decimalDegrees = degrees + minutes / 60;
end