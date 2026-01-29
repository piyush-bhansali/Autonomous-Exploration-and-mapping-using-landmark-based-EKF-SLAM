% MATLAB Script to Compare EKF Localization vs Gazebo Ground Truth
% Reads CSV data and plots trajectory comparison with error analysis
%
% Usage:
%   1. Run your ROS2 simulation to generate the CSV file
%   2. Update the csv_file path below to point to your data file
%   3. Run this script in MATLAB

clear; close all; clc;

%% Configuration
% Update this path to your CSV file location
csv_file = './submaps/tb3_1/ekf_vs_groundtruth.csv';

% Check if file exists
if ~isfile(csv_file)
    error('CSV file not found: %s\nPlease run the simulation first to generate data.', csv_file);
end

%% Load Data
fprintf('Loading data from: %s\n', csv_file);
data = readtable(csv_file);

% Extract columns
timestamp = data.timestamp;
ekf_x = data.ekf_x;
ekf_y = data.ekf_y;
ekf_theta = data.ekf_theta;
gt_x = data.gt_x;
gt_y = data.gt_y;
gt_theta = data.gt_theta;
pos_error = data.pos_error;
orient_error = data.orient_error;

% Normalize timestamp to start from 0
timestamp = timestamp - timestamp(1);

fprintf('Loaded %d data points\n', length(timestamp));
fprintf('Duration: %.2f seconds\n', timestamp(end));

%% Statistics
fprintf('\n=== ERROR STATISTICS ===\n');
fprintf('Position Error:\n');
fprintf('  Mean: %.4f m\n', mean(pos_error));
fprintf('  RMSE: %.4f m\n', sqrt(mean(pos_error.^2)));
fprintf('  Max:  %.4f m\n', max(pos_error));
fprintf('  Std:  %.4f m\n', std(pos_error));
fprintf('\nOrientation Error:\n');
fprintf('  Mean: %.4f rad (%.2f deg)\n', mean(orient_error), rad2deg(mean(orient_error)));
fprintf('  RMSE: %.4f rad (%.2f deg)\n', sqrt(mean(orient_error.^2)), rad2deg(sqrt(mean(orient_error.^2))));
fprintf('  Max:  %.4f rad (%.2f deg)\n', max(orient_error), rad2deg(max(orient_error)));
fprintf('  Std:  %.4f rad (%.2f deg)\n', std(orient_error), rad2deg(std(orient_error)));

%% Plot 1: 2D Trajectory Comparison
figure('Name', 'EKF vs Ground Truth - Trajectory', 'Position', [100, 100, 1200, 800]);

subplot(2, 2, 1);
plot(gt_x, gt_y, 'b-', 'LineWidth', 2, 'DisplayName', 'Ground Truth');
hold on;
plot(ekf_x, ekf_y, 'r--', 'LineWidth', 1.5, 'DisplayName', 'EKF Estimate');
plot(gt_x(1), gt_y(1), 'go', 'MarkerSize', 12, 'LineWidth', 2, 'DisplayName', 'Start');
plot(gt_x(end), gt_y(end), 'rs', 'MarkerSize', 12, 'LineWidth', 2, 'DisplayName', 'End');
grid on;
xlabel('X Position (m)', 'FontSize', 12);
ylabel('Y Position (m)', 'FontSize', 12);
title('Robot Trajectory Comparison', 'FontSize', 14, 'FontWeight', 'bold');
legend('Location', 'best', 'FontSize', 10);
axis equal;

% Plot 2: Position Error Over Time
subplot(2, 2, 2);
plot(timestamp, pos_error, 'r-', 'LineWidth', 1.5);
hold on;
yline(mean(pos_error), 'b--', 'LineWidth', 2, 'DisplayName', sprintf('Mean = %.3f m', mean(pos_error)));
yline(sqrt(mean(pos_error.^2)), 'g--', 'LineWidth', 2, 'DisplayName', sprintf('RMSE = %.3f m', sqrt(mean(pos_error.^2))));
grid on;
xlabel('Time (s)', 'FontSize', 12);
ylabel('Position Error (m)', 'FontSize', 12);
title('Position Error vs Time', 'FontSize', 14, 'FontWeight', 'bold');
legend('Error', 'Mean', 'RMSE', 'Location', 'best', 'FontSize', 10);

% Plot 3: Orientation Error Over Time
subplot(2, 2, 3);
plot(timestamp, rad2deg(orient_error), 'b-', 'LineWidth', 1.5);
hold on;
yline(rad2deg(mean(orient_error)), 'r--', 'LineWidth', 2, 'DisplayName', sprintf('Mean = %.2f°', rad2deg(mean(orient_error))));
yline(rad2deg(sqrt(mean(orient_error.^2))), 'g--', 'LineWidth', 2, 'DisplayName', sprintf('RMSE = %.2f°', rad2deg(sqrt(mean(orient_error.^2)))));
grid on;
xlabel('Time (s)', 'FontSize', 12);
ylabel('Orientation Error (deg)', 'FontSize', 12);
title('Orientation Error vs Time', 'FontSize', 14, 'FontWeight', 'bold');
legend('Error', 'Mean', 'RMSE', 'Location', 'best', 'FontSize', 10);

% Plot 4: X and Y Position Comparison
subplot(2, 2, 4);
plot(timestamp, gt_x, 'b-', 'LineWidth', 2, 'DisplayName', 'GT X');
hold on;
plot(timestamp, ekf_x, 'b--', 'LineWidth', 1.5, 'DisplayName', 'EKF X');
plot(timestamp, gt_y, 'r-', 'LineWidth', 2, 'DisplayName', 'GT Y');
plot(timestamp, ekf_y, 'r--', 'LineWidth', 1.5, 'DisplayName', 'EKF Y');
grid on;
xlabel('Time (s)', 'FontSize', 12);
ylabel('Position (m)', 'FontSize', 12);
title('X and Y Position Components', 'FontSize', 14, 'FontWeight', 'bold');
legend('Location', 'best', 'FontSize', 10);

sgtitle('EKF Localization vs Gazebo Ground Truth', 'FontSize', 16, 'FontWeight', 'bold');

%% Plot 2: Detailed Error Analysis
figure('Name', 'Error Analysis', 'Position', [150, 150, 1200, 600]);

% X Error
subplot(2, 3, 1);
x_error = ekf_x - gt_x;
plot(timestamp, x_error, 'b-', 'LineWidth', 1.5);
hold on;
yline(0, 'k--', 'LineWidth', 1);
yline(mean(x_error), 'r--', 'LineWidth', 2, 'DisplayName', sprintf('Mean = %.4f m', mean(x_error)));
grid on;
xlabel('Time (s)', 'FontSize', 12);
ylabel('X Error (m)', 'FontSize', 12);
title('X Position Error', 'FontSize', 12, 'FontWeight', 'bold');
legend('Location', 'best');

% Y Error
subplot(2, 3, 2);
y_error = ekf_y - gt_y;
plot(timestamp, y_error, 'r-', 'LineWidth', 1.5);
hold on;
yline(0, 'k--', 'LineWidth', 1);
yline(mean(y_error), 'b--', 'LineWidth', 2, 'DisplayName', sprintf('Mean = %.4f m', mean(y_error)));
grid on;
xlabel('Time (s)', 'FontSize', 12);
ylabel('Y Error (m)', 'FontSize', 12);
title('Y Position Error', 'FontSize', 12, 'FontWeight', 'bold');
legend('Location', 'best');

% Theta Error
subplot(2, 3, 3);
theta_error = ekf_theta - gt_theta;
% Wrap angle error to [-pi, pi]
theta_error = atan2(sin(theta_error), cos(theta_error));
plot(timestamp, rad2deg(theta_error), 'g-', 'LineWidth', 1.5);
hold on;
yline(0, 'k--', 'LineWidth', 1);
yline(rad2deg(mean(theta_error)), 'm--', 'LineWidth', 2, 'DisplayName', sprintf('Mean = %.2f°', rad2deg(mean(theta_error))));
grid on;
xlabel('Time (s)', 'FontSize', 12);
ylabel('Theta Error (deg)', 'FontSize', 12);
title('Orientation Error', 'FontSize', 12, 'FontWeight', 'bold');
legend('Location', 'best');

% Position Error Distribution
subplot(2, 3, 4);
histogram(pos_error, 30, 'FaceColor', 'b', 'EdgeColor', 'k');
hold on;
xline(mean(pos_error), 'r--', 'LineWidth', 2, 'DisplayName', 'Mean');
xline(sqrt(mean(pos_error.^2)), 'g--', 'LineWidth', 2, 'DisplayName', 'RMSE');
grid on;
xlabel('Position Error (m)', 'FontSize', 12);
ylabel('Count', 'FontSize', 12);
title('Position Error Distribution', 'FontSize', 12, 'FontWeight', 'bold');
legend('Location', 'best');

% Orientation Error Distribution
subplot(2, 3, 5);
histogram(rad2deg(orient_error), 30, 'FaceColor', 'r', 'EdgeColor', 'k');
hold on;
xline(rad2deg(mean(orient_error)), 'b--', 'LineWidth', 2, 'DisplayName', 'Mean');
xline(rad2deg(sqrt(mean(orient_error.^2))), 'g--', 'LineWidth', 2, 'DisplayName', 'RMSE');
grid on;
xlabel('Orientation Error (deg)', 'FontSize', 12);
ylabel('Count', 'FontSize', 12);
title('Orientation Error Distribution', 'FontSize', 12, 'FontWeight', 'bold');
legend('Location', 'best');

% Error Scatter Plot
subplot(2, 3, 6);
scatter(pos_error, rad2deg(orient_error), 20, timestamp, 'filled');
colorbar;
xlabel('Position Error (m)', 'FontSize', 12);
ylabel('Orientation Error (deg)', 'FontSize', 12);
title('Position vs Orientation Error (colored by time)', 'FontSize', 12, 'FontWeight', 'bold');
grid on;

sgtitle('Detailed Error Analysis', 'FontSize', 16, 'FontWeight', 'bold');

%% Plot 3: Zoomed Trajectory Sections (if data is long enough)
if length(timestamp) > 100
    figure('Name', 'Trajectory Detail', 'Position', [200, 200, 1200, 400]);

    % Beginning
    subplot(1, 3, 1);
    n_points = min(50, floor(length(timestamp)/3));
    plot(gt_x(1:n_points), gt_y(1:n_points), 'b-', 'LineWidth', 2);
    hold on;
    plot(ekf_x(1:n_points), ekf_y(1:n_points), 'r--', 'LineWidth', 1.5);
    plot(gt_x(1), gt_y(1), 'go', 'MarkerSize', 10, 'LineWidth', 2);
    grid on;
    xlabel('X (m)'); ylabel('Y (m)');
    title('Start of Trajectory');
    legend('Ground Truth', 'EKF', 'Start', 'Location', 'best');
    axis equal;

    % Middle
    subplot(1, 3, 2);
    mid_idx = floor(length(timestamp)/2);
    idx_range = max(1, mid_idx-25):min(length(timestamp), mid_idx+25);
    plot(gt_x(idx_range), gt_y(idx_range), 'b-', 'LineWidth', 2);
    hold on;
    plot(ekf_x(idx_range), ekf_y(idx_range), 'r--', 'LineWidth', 1.5);
    grid on;
    xlabel('X (m)'); ylabel('Y (m)');
    title('Middle of Trajectory');
    legend('Ground Truth', 'EKF', 'Location', 'best');
    axis equal;

    % End
    subplot(1, 3, 3);
    plot(gt_x(end-n_points:end), gt_y(end-n_points:end), 'b-', 'LineWidth', 2);
    hold on;
    plot(ekf_x(end-n_points:end), ekf_y(end-n_points:end), 'r--', 'LineWidth', 1.5);
    plot(gt_x(end), gt_y(end), 'rs', 'MarkerSize', 10, 'LineWidth', 2);
    grid on;
    xlabel('X (m)'); ylabel('Y (m)');
    title('End of Trajectory');
    legend('Ground Truth', 'EKF', 'End', 'Location', 'best');
    axis equal;

    sgtitle('Trajectory Detail Views', 'FontSize', 16, 'FontWeight', 'bold');
end

fprintf('\nPlotting complete!\n');
