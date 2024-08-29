% Radius of the Earth in meters
R = 6378137;

origin_lat = 35.770764;
origin_lon = -78.674802;

points = readtable("C:\Users\malin\Documents\GitHub\project-oval\Data\position_data_logger_test5.csv");

figure;

% Create subplots
subplot(2, 1, 1);
title('RTK, and ZED Positions');
xlabel('X (meters)');
ylabel('Y (meters)');
hold on;

subplot(2, 1, 2);
title('Combined Percent Error of Position');
xlabel('Point Index');
ylabel('Percent Error (%)');
hold on;

numRows = height(points);

% Arrays to store points for path plotting
rtk_path = [];
gps_path = [];
ml_path = [];
zed_path = [];

% Arrays to store combined percent errors
gps_combined_error = [];
ml_combined_error = [];
zed_combined_error = [];

for i = 1:numRows
    fprintf("Next Point\n");
    rtk_lat = points.swift_latitude(i);
    rtk_lon = points.swift_longitude(i);
    gps_lat = points.gps_latitude(i);
    gps_lon = points.gps_longitude(i);
    ml_lat = points.ml_latitude(i);
    ml_lon = points.ml_longitude(i);
    zed_x = points.zed_x(i);
    zed_y = points.zed_y(i);

    if rtk_lat ~= 0
        rtk_point = latlon_to_xy(rtk_lat, rtk_lon, origin_lat, origin_lon, R);
        gps_point = latlon_to_xy(gps_lat, gps_lon, origin_lat, origin_lon, R);
        ml_point = latlon_to_xy(ml_lat, ml_lon, origin_lat, origin_lon, R);
        zed_point = convert_zed(zed_x, zed_y);

        % Append points to paths
        rtk_path = [rtk_path; rtk_point];
        gps_path = [gps_path; gps_point];
        ml_path = [ml_path; ml_point];
        zed_path = [zed_path; zed_point];

        % Calculate combined percent errors
        gps_error = combined_percent_error(rtk_point, gps_point);
        ml_error = combined_percent_error(rtk_point, ml_point);
        zed_error = combined_percent_error(rtk_point, zed_point);

        if gps_error > 100
            gps_error = 0;
        end
        if ml_error > 100
            ml_error = 0;
        end
        if zed_error > 100
            zed_error = 0;
        end

        gps_combined_error = [gps_combined_error; gps_error];
        ml_combined_error = [ml_combined_error; ml_error];
        zed_combined_error = [zed_combined_error; zed_error];

        % Clear subplots
        subplot(2, 1, 1);
        cla;
        hold on;
        
        % Plot the paths up to the current point
        
        %plot(gps_path(:, 1), gps_path(:, 2), 'b-', 'LineWidth', 1);
        %plot(ml_path(:, 1), ml_path(:, 2), 'g-', 'LineWidth', 1);
        plot(zed_path(:, 1), zed_path(:, 2), 'k-', 'LineWidth', 1);

        plot(rtk_path(:, 1), rtk_path(:, 2), 'r-', 'LineWidth', 1);
        % Plot the most recent points with distinct markers
        %plot(rtk_path(end, 1), rtk_path(end, 2), 'ro', 'MarkerSize', 6, 'MarkerFaceColor', 'r');
        %plot(gps_path(end, 1), gps_path(end, 2), 'bo', 'MarkerSize', 6, 'MarkerFaceColor', 'b');
        %plot(ml_path(end, 1), ml_path(end, 2), 'go', 'MarkerSize', 6, 'MarkerFaceColor', 'g');
        %plot(zed_path(end, 1), zed_path(end, 2), 'ko', 'MarkerSize', 6, 'MarkerFaceColor', 'k');

        % Clear the second subplot
        subplot(2, 1, 2);
        cla;
        hold on;
        
        % Plot the combined percent errors
        plot(gps_combined_error, 'b-', 'LineWidth', 1);
        plot(ml_combined_error, 'g-', 'LineWidth', 1);
        plot(zed_combined_error, 'k-', 'LineWidth', 1);
        legend('GPS', 'ML', 'ZED');

        % Calculate and display statistics
        avg_gps_error = mean(gps_combined_error);
        avg_ml_error = mean(ml_combined_error);
        avg_zed_error = mean(zed_combined_error);
        
        fprintf('Average GPS Percent Error: %.2f%%\n', avg_gps_error);
        fprintf('Average ML Percent Error: %.2f%%\n', avg_ml_error);
        fprintf('Average ZED Percent Error: %.2f%%\n', avg_zed_error);
        
        % Calculate R-squared
        rtk_distances = sqrt(rtk_path(:, 1).^2 + rtk_path(:, 2).^2);
        gps_distances = sqrt(gps_path(:, 1).^2 + gps_path(:, 2).^2);
        ml_distances = sqrt(ml_path(:, 1).^2 + ml_path(:, 2).^2);
        zed_distances = sqrt(zed_path(:, 1).^2 + zed_path(:, 2).^2);

        % R-squared for GPS
        ss_res_gps = sum((rtk_distances - gps_distances).^2);
        ss_tot_gps = sum((rtk_distances - mean(rtk_distances)).^2);
        r_squared_gps = 1 - (ss_res_gps / ss_tot_gps);

        % R-squared for ML
        ss_res_ml = sum((rtk_distances - ml_distances).^2);
        ss_tot_ml = sum((rtk_distances - mean(rtk_distances)).^2);
        r_squared_ml = 1 - (ss_res_ml / ss_tot_ml);

        % R-squared for ZED
        ss_res_zed = sum((rtk_distances - zed_distances).^2);
        ss_tot_zed = sum((rtk_distances - mean(rtk_distances)).^2);
        r_squared_zed = 1 - (ss_res_zed / ss_tot_zed);

        fprintf('R-squared (GPS): %.4f\n', r_squared_gps);
        fprintf('R-squared (ML): %.4f\n', r_squared_ml);
        fprintf('R-squared (ZED): %.4f\n', r_squared_zed);
        
        % Calculate variance and standard deviation
        var_gps_error = var(gps_combined_error);
        std_gps_error = std(gps_combined_error);
        
        var_ml_error = var(ml_combined_error);
        std_ml_error = std(ml_combined_error);

        var_zed_error = var(zed_combined_error);
        std_zed_error = std(zed_combined_error);

        fprintf('GPS Error Variance: %.4f\n', var_gps_error);
        fprintf('GPS Error Standard Deviation: %.4f\n', std_gps_error);
        fprintf('ML Error Variance: %.4f\n', var_ml_error);
        fprintf('ML Error Standard Deviation: %.4f\n', std_ml_error);
        fprintf('ZED Error Variance: %.4f\n', var_zed_error);
        fprintf('ZED Error Standard Deviation: %.4f\n', std_zed_error);

        pause(0.1);
    end
end

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

    point = [y, x];
end

% Haversine function to calculate distance
function distance = haversine(lat, lon, origin_lat, origin_lon, R)
    d_lat = deg2rad(lat - origin_lat);
    d_lon = deg2rad(lon - origin_lon);

    a = sin(d_lat / 2)^2 + cos(deg2rad(origin_lat)) * cos(deg2rad(lat)) * sin(d_lon / 2)^2;
    c = 2 * atan2(sqrt(a), sqrt(1 - a));

    distance = R * c;
end

function point = gps_converter(original_point)
    degrees = floor(original_point / 100);
    minutes = original_point - (degrees * 100);
    point = degrees + (minutes / 60);

    if point > 50
        point = point * -1.0;
    end
end

% Function to calculate combined percent error
function error = combined_percent_error(true_point, measured_point)
    distance_true = sqrt(true_point(1)^2 + true_point(2)^2);
    distance_measured = sqrt(measured_point(1)^2 + measured_point(2)^2);
    error = abs((distance_true - distance_measured) / distance_true) * 100;
end

function point = convert_zed(x, y)
    
    origin = [35, 76];

    x = -1 * x;
    y = y;
    translated_point = [x, y];

    theta = deg2rad(14.5);

    R = [cos(theta) sin(theta); -sin(theta) cos(theta)];

    rotated_point = (R * translated_point')';

    point = rotated_point + origin;
end
