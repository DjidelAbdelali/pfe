% =========================================================================
% 3D SIMBODY / SIMSCAPE MULTIBODY ROBOT SIMULATION SUITE
% Simulates the 3D CAD Multi-Body Delta Robot Mechanism with Sliding Control
% =========================================================================

clearvars -except artifactDir;
clc;
close all;

% Locate 3D Modeling folder
scriptPath = fileparts(mfilename('fullpath'));
if endsWith(scriptPath, '3D Modeling')
    folder3D = scriptPath;
    rootDir = fileparts(scriptPath);
else
    folder3D = fullfile(pwd, '3D Modeling');
    rootDir = pwd;
end

% Ensure root and 3D subfolders are added to MATLAB path
addpath(genpath(rootDir));

% Set up artifact directory
if ~exist('artifactDir', 'var') || isempty(artifactDir)
    artifactDir = 'C:\Users\Dchak\.gemini\antigravity-ide\brain\3466a3e1-e023-47aa-bc82-ef4159f6d174';
end
if ~exist(artifactDir, 'dir')
    artifactDir = fullfile(folder3D, 'results');
end
if ~exist(artifactDir, 'dir')
    mkdir(artifactDir);
end

fprintf('=====================================================\n');
fprintf('  STARTING 3D SIMBODY / SIMSCAPE MULTIBODY ROBOT SIMULATION\n');
fprintf('  3D Modeling Directory: %s\n', folder3D);
fprintf('=====================================================\n\n');

%% Load 3D Simscape Parameters & Control gains
curDir = pwd;
cd(folder3D);

if exist('RobotDelta_4degree_DataFile.m', 'file')
    run('RobotDelta_4degree_DataFile.m');
    fprintf('Loaded RobotDelta_4degree_DataFile.m\n');
else
    error('RobotDelta_4degree_DataFile.m not found in %s', folder3D);
end

h = 0.001;
tf = 0.5;
n = double(ceil(tf/h));
kp = 16;
kd = 1.0245;
mu = 0.5308;
Mb = 1.999e-4;
ks = 641.6;

% Load 3D model
model3D = 'simulation17PDfract';
load_system(model3D);
fprintf('Loaded 3D Simscape Model: %s\n', model3D);

%% Execute 3D Multibody Simulation
fprintf('Executing 3D Multi-Body Dynamics solver...\n');
simOut = sim(model3D, 'StopTime', num2str(tf));
cd(curDir);
fprintf('3D Multibody Simulation Completed Successfully!\n\n');

%% Extract 3D Motion Signals
T2_raw       = getValRaw(simOut, 'T2');
P2_raw       = getValRaw(simOut, 'P2');        % Actual end-effector position [x, y, z]
Pdes2_raw    = getValRaw(simOut, 'Pdes2');     % Desired end-effector position [x_d, y_d, z_d]
alpha_raw    = getValRaw(simOut, 'alpha');     % Joint angles [alpha1, alpha2, alpha3]
alphades_raw = getValRaw(simOut, 'alphades');  % Desired joint angles [alpha1_d, alpha2_d, alpha3_d]

% Extract 1D Time vector
T2 = squeeze(T2_raw(1,1,:));

% Format 3D position & angle matrices to [N_steps x 3]
P2       = squeeze(P2_raw)';
Pdes2    = squeeze(Pdes2_raw)';
alpha    = squeeze(alpha_raw)';
alphades = squeeze(alphades_raw)';

%% -------------------------------------------------------------------------
%% FIGURE 3D-1: 3D Spatial Trajectory of Delta Robot End-Effector
%% -------------------------------------------------------------------------
fig3D_1 = figure('Name', '3D Spatial Trajectory', 'Color', 'w', 'Position', [100 100 900 700], 'Visible', 'off');
grid on; box on; hold on;
plot3(Pdes2(:,1), Pdes2(:,2), Pdes2(:,3), 'r--', 'LineWidth', 2.5, 'DisplayName', 'Desired 3D Path');
plot3(P2(:,1), P2(:,2), P2(:,3), 'b-', 'LineWidth', 1.8, 'DisplayName', 'Realized 3D Robot Path');
xlabel('X Position [m]', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('Y Position [m]', 'FontSize', 12, 'FontWeight', 'bold');
zlabel('Z Position [m]', 'FontSize', 12, 'FontWeight', 'bold');
title('3D Simscape Multibody Delta Robot End-Effector Motion', 'FontSize', 14, 'FontWeight', 'bold');
legend('Location', 'best', 'FontSize', 11);
view(45, 30);

saveas(fig3D_1, fullfile(artifactDir, 'fig3D_1_spatial_trajectory.png'));
saveas(fig3D_1, fullfile(folder3D, 'fig3D_1_spatial_trajectory.png'));
close(fig3D_1);
fprintf('Saved 3D Figure 1: fig3D_1_spatial_trajectory.png\n');

%% -------------------------------------------------------------------------
%% FIGURE 3D-2: Joint Angles Dynamics (\alpha_1, \alpha_2, \alpha_3)
%% -------------------------------------------------------------------------
fig3D_2 = figure('Name', '3D Joint Angles', 'Color', 'w', 'Position', [100 100 1000 750], 'Visible', 'off');

subplot(3,1,1); hold on; grid on; box on;
plot(T2, alphades(:,1), 'r--', 'LineWidth', 1.5, 'DisplayName', '\alpha_{1,des}');
plot(T2, alpha(:,1), 'b-', 'LineWidth', 1.5, 'DisplayName', '\alpha_{1,actual}');
ylabel('Angle \alpha_1 [rad]', 'FontSize', 11, 'FontWeight', 'bold');
title('Arm 1 Joint Angle Motion', 'FontSize', 13, 'FontWeight', 'bold');
legend('Location', 'best');

subplot(3,1,2); hold on; grid on; box on;
plot(T2, alphades(:,2), 'r--', 'LineWidth', 1.5, 'DisplayName', '\alpha_{2,des}');
plot(T2, alpha(:,2), 'g-', 'LineWidth', 1.5, 'DisplayName', '\alpha_{2,actual}');
ylabel('Angle \alpha_2 [rad]', 'FontSize', 11, 'FontWeight', 'bold');
title('Arm 2 Joint Angle Motion', 'FontSize', 13, 'FontWeight', 'bold');
legend('Location', 'best');

subplot(3,1,3); hold on; grid on; box on;
plot(T2, alphades(:,3), 'r--', 'LineWidth', 1.5, 'DisplayName', '\alpha_{3,des}');
plot(T2, alpha(:,3), 'm-', 'LineWidth', 1.5, 'DisplayName', '\alpha_{3,actual}');
xlabel('Time [s]', 'FontSize', 11, 'FontWeight', 'bold');
ylabel('Angle \alpha_3 [rad]', 'FontSize', 11, 'FontWeight', 'bold');
title('Arm 3 Joint Angle Motion', 'FontSize', 13, 'FontWeight', 'bold');
legend('Location', 'best');

saveas(fig3D_2, fullfile(artifactDir, 'fig3D_2_joint_angles.png'));
saveas(fig3D_2, fullfile(folder3D, 'fig3D_2_joint_angles.png'));
close(fig3D_2);
fprintf('Saved 3D Figure 2: fig3D_2_joint_angles.png\n');

%% -------------------------------------------------------------------------
%% FIGURE 3D-3: 3D Tracking Errors over Time
%% -------------------------------------------------------------------------
fig3D_3 = figure('Name', '3D Tracking Errors', 'Color', 'w', 'Position', [100 100 1000 650], 'Visible', 'off');

eX = Pdes2(:,1) - P2(:,1);
eY = Pdes2(:,2) - P2(:,2);
eZ = Pdes2(:,3) - P2(:,3);

plot(T2, eX, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Error e_x(t)'); hold on; grid on; box on;
plot(T2, eY, 'g-', 'LineWidth', 1.5, 'DisplayName', 'Error e_y(t)');
plot(T2, eZ, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Error e_z(t)');
xlabel('Time [s]', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('Tracking Error [m]', 'FontSize', 12, 'FontWeight', 'bold');
title('3D End-Effector Spatial Tracking Errors', 'FontSize', 14, 'FontWeight', 'bold');
legend('Location', 'best', 'FontSize', 10);

saveas(fig3D_3, fullfile(artifactDir, 'fig3D_3_position_tracking_3d.png'));
saveas(fig3D_3, fullfile(folder3D, 'fig3D_3_position_tracking_3d.png'));
close(fig3D_3);
fprintf('Saved 3D Figure 3: fig3D_3_position_tracking_3d.png\n');

%% Export 3D Simulation Data MAT and CSV
save(fullfile(folder3D, 'simbody_3d_results.mat'), 'T2', 'P2', 'Pdes2', 'alpha', 'alphades');
save(fullfile(artifactDir, 'simbody_3d_results.mat'), 'T2', 'P2', 'Pdes2', 'alpha', 'alphades');

T3D = table(T2, P2(:,1), P2(:,2), P2(:,3), Pdes2(:,1), Pdes2(:,2), Pdes2(:,3), ...
    'VariableNames', {'Time', 'X_actual', 'Y_actual', 'Z_actual', 'X_desired', 'Y_desired', 'Z_desired'});
writetable(T3D, fullfile(folder3D, 'simbody_3d_results.csv'));
writetable(T3D, fullfile(artifactDir, 'simbody_3d_results.csv'));
fprintf('Exported 3D results to simbody_3d_results.csv and simbody_3d_results.mat\n');

fprintf('\n=====================================================\n');
fprintf('  3D SIMBODY SIMULATION SUITE COMPLETED SUCCESSFULLY!\n');
fprintf('=====================================================\n');

function val = getValRaw(simOut, varName)
    try
        if isprop(simOut, varName) || isfield(simOut, varName)
            v = simOut.(varName);
        else
            v = evalin('base', varName);
        end
    catch
        v = evalin('base', varName);
    end
    
    if isa(v, 'timeseries')
        val = v.Data;
    elseif isstruct(v) && isfield(v, 'signals')
        val = v.signals.values;
    else
        val = double(v);
    end
end
