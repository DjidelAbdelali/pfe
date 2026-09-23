% =========================================================================
% MASTER SIMULATION & COMPARATIVE ANALYSIS RUNNER
% PFE Final: Comparison of SMC Control Strategies (PD/PD-F, S2 Surface, S1+Sat, S1+DHRL)
% =========================================================================

clearvars -except artifactDir;
clc;
close all;

% Determine root directory of workspace regardless of current folder
scriptPath = fileparts(mfilename('fullpath'));
if endsWith(scriptPath, '3D Modeling')
    rootDir = fileparts(scriptPath);
elseif isempty(scriptPath)
    rootDir = pwd;
else
    rootDir = scriptPath;
end

% Ensure all subdirectories are added to MATLAB path
addpath(genpath(rootDir));

% Set up output directory
if ~exist('artifactDir', 'var') || isempty(artifactDir)
    artifactDir = 'C:\Users\Dchak\.gemini\antigravity-ide\brain\3466a3e1-e023-47aa-bc82-ef4159f6d174';
end
if ~exist(artifactDir, 'dir')
    artifactDir = fullfile(rootDir, 'results');
end
if ~exist(artifactDir, 'dir')
    mkdir(artifactDir);
end

folder3D = fullfile(rootDir, '3D Modeling');
if ~exist(folder3D, 'dir')
    mkdir(folder3D);
end

fprintf('=====================================================\n');
fprintf('  STARTING COMPREHENSIVE SIMULATION SUITE FOR ALL MODELS\n');
fprintf('  Workspace Root Directory: %s\n', rootDir);
fprintf('=====================================================\n\n');

%% Common parameters setup
Mb = 1.999e-4;
ks = 641.6;
h = 0.001;
tf = 0.5;
n = double(ceil(tf/h));

results = struct();

%% -------------------------------------------------------------------------
%% 1. MODEL 1: PD / PD-F + SMC (TDC_SMC_PD_S10x2810x29)
%% -------------------------------------------------------------------------
fprintf('[1/4] Simulating Model 1: PD / PD-F + SMC...\n');
try
    curDir = pwd;
    cd(fullfile(rootDir, 'PD et PD-F'));
    run('PD_final.m');
    Mb = 1.999e-4; ks = 641.6; h = 0.001; tf = 0.5; n = double(ceil(tf/h));
    simOut1 = sim('TDC_SMC_PD_S10x2810x29', 'StopTime', num2str(tf));
    cd(curDir);
    
    results.M1.t = getVal(simOut1, 't');
    results.M1.xref = getVal(simOut1, 'xref');
    results.M1.xpd  = getVal(simOut1, 'xpd');
    results.M1.zref = getVal(simOut1, 'zref');
    results.M1.zpd  = getVal(simOut1, 'zpd');
    results.M1.u1   = getVal(simOut1, 'coup');
    results.M1.u2   = getVal(simOut1, 'coup1');
    results.M1.u3   = getVal(simOut1, 'coup2');
    results.M1.s1   = getVal(simOut1, 'alp');
    results.M1.s2   = getVal(simOut1, 'alp1');
    results.M1.s3   = getVal(simOut1, 'alp2');
    results.M1.name = 'PD / PD-F SMC';
    fprintf('   --> Model 1 completed successfully.\n');
catch ME
    cd(curDir);
    fprintf('   --> ERROR in Model 1: %s\n', ME.message);
end

%% -------------------------------------------------------------------------
%% 2. MODEL 2: Surface S2 (S2_surfs)
%% -------------------------------------------------------------------------
fprintf('[2/4] Simulating Model 2: Surface S2...\n');
try
    curDir = pwd;
    cd(fullfile(rootDir, 'Surface de comparaison'));
    run('S2_surf.m');
    Mb = 1.999e-4; ks = 641.6; h = 0.001; tf = 0.5; n = double(ceil(tf/h));
    simOut2 = sim('S2_surfs', 'StopTime', num2str(tf));
    cd(curDir);
    
    results.M2.t = getVal(simOut2, 't');
    results.M2.xref = getVal(simOut2, 'xref');
    results.M2.xpd  = getVal(simOut2, 'xpd');
    results.M2.zref = getVal(simOut2, 'zref');
    results.M2.zpd  = getVal(simOut2, 'zpd');
    results.M2.u1   = getVal(simOut2, 'coup');
    results.M2.u2   = getVal(simOut2, 'coup1');
    results.M2.u3   = getVal(simOut2, 'coup2');
    results.M2.s1   = getVal(simOut2, 'alp');
    results.M2.s2   = getVal(simOut2, 'alp1');
    results.M2.s3   = getVal(simOut2, 'alp2');
    results.M2.name = 'Surface S2';
    fprintf('   --> Model 2 completed successfully.\n');
catch ME
    cd(curDir);
    fprintf('   --> ERROR in Model 2: %s\n', ME.message);
end

%% -------------------------------------------------------------------------
%% 3. MODEL 3: Proposed Surface S1 + Saturation (S1_sats)
%% -------------------------------------------------------------------------
fprintf('[3/4] Simulating Model 3: Proposed Surface S1 + Saturation...\n');
try
    curDir = pwd;
    cd(fullfile(rootDir, 'Surface proposee'));
    run('S1_sat.m');
    simOut3 = sim('S1_sats', 'StopTime', num2str(tf));
    cd(curDir);
    
    results.M3.t = getVal(simOut3, 't1');
    results.M3.xref = getVal(simOut3, 'xref1');
    results.M3.xpd  = getVal(simOut3, 'xpd1');
    results.M3.zref = getVal(simOut3, 'zref1');
    results.M3.zpd  = getVal(simOut3, 'zpd1');
    results.M3.u1   = getVal(simOut3, 'coup11');
    results.M3.u2   = getVal(simOut3, 'coup12');
    results.M3.u3   = getVal(simOut3, 'coup13');
    results.M3.s1   = getVal(simOut3, 'alp11');
    results.M3.s2   = getVal(simOut3, 'alp12');
    results.M3.s3   = getVal(simOut3, 'alp13');
    results.M3.name = 'Surface S1 + Sat';
    fprintf('   --> Model 3 completed successfully.\n');
catch ME
    cd(curDir);
    fprintf('   --> ERROR in Model 3: %s\n', ME.message);
end

%% -------------------------------------------------------------------------
%% 4. MODEL 4: Proposed Surface S1 + DHRL (S1_DHRLs)
%% -------------------------------------------------------------------------
fprintf('[4/4] Simulating Model 4: Proposed Surface S1 + DHRL...\n');
try
    curDir = pwd;
    cd(fullfile(rootDir, 'Surface proposee'));
    run('S1_DHRL.m');
    simOut4 = sim('S1_DHRLs', 'StopTime', num2str(tf));
    cd(curDir);
    
    results.M4.t = getVal(simOut4, 't2');
    results.M4.xref = getVal(simOut4, 'xref2');
    results.M4.xpd  = getVal(simOut4, 'xpd2');
    results.M4.zref = getVal(simOut4, 'zref2');
    results.M4.zpd  = getVal(simOut4, 'zpd2');
    results.M4.u1   = getVal(simOut4, 'coup21');
    results.M4.u2   = getVal(simOut4, 'coup22');
    results.M4.u3   = getVal(simOut4, 'coup23');
    results.M4.s1   = getVal(simOut4, 'alp21');
    results.M4.s2   = getVal(simOut4, 'alp22');
    results.M4.s3   = getVal(simOut4, 'alp23');
    results.M4.name = 'Surface S1 + DHRL';
    fprintf('   --> Model 4 completed successfully.\n');
catch ME
    cd(curDir);
    fprintf('   --> ERROR in Model 4: %s\n', ME.message);
end

%% =========================================================================
%% CALCULATE PERFORMANCE METRICS
%% =========================================================================
fprintf('\nCalculating quantitative performance metrics...\n');

modelsKey = {'M1', 'M2', 'M3', 'M4'};
metrics = struct();

for i = 1:length(modelsKey)
    k = modelsKey{i};
    if isfield(results, k) && ~isempty(results.(k).t)
        t = results.(k).t;
        dt = mean(diff(t));
        
        ex = results.(k).xref - results.(k).xpd;
        ez = results.(k).zref - results.(k).zpd;
        e_total = sqrt(ex.^2 + ez.^2);
        
        % Error indices
        metrics.(k).IAE = sum(abs(e_total)) * dt;
        metrics.(k).ISE = sum(e_total.^2) * dt;
        metrics.(k).ITAE = sum(t .* abs(e_total)) * dt;
        metrics.(k).RMSE = sqrt(mean(e_total.^2));
        metrics.(k).PeakError = max(abs(e_total));
        
        % Control effort indices
        u1 = results.(k).u1;
        u2 = results.(k).u2;
        u3 = results.(k).u3;
        u_norm = sqrt(u1.^2 + u2.^2 + u3.^2);
        
        metrics.(k).PeakControl = max(u_norm);
        metrics.(k).RMSControl  = sqrt(mean(u_norm.^2));
        
        % Chattering metric (Total Variation of control input)
        metrics.(k).Chattering = sum(abs(diff(u1))) + sum(abs(diff(u2))) + sum(abs(diff(u3)));
    end
end

% Display Metrics Table
fprintf('\n=======================================================================================================\n');
fprintf('%-20s | %-10s | %-10s | %-10s | %-10s | %-12s | %-12s\n', ...
    'Control Strategy', 'IAE', 'ISE', 'ITAE', 'RMSE', 'Peak Control', 'Chattering');
fprintf('-------------------------------------------------------------------------------------------------------\n');
for i = 1:length(modelsKey)
    k = modelsKey{i};
    if isfield(metrics, k)
        fprintf('%-20s | %-10.6f | %-10.6f | %-10.6f | %-10.6f | %-12.4f | %-12.4f\n', ...
            results.(k).name, metrics.(k).IAE, metrics.(k).ISE, metrics.(k).ITAE, ...
            metrics.(k).RMSE, metrics.(k).PeakControl, metrics.(k).Chattering);
    end
end
fprintf('=======================================================================================================\n\n');

%% =========================================================================
%% GENERATE COMPARATIVE FIGURES
%% =========================================================================
set(0, 'DefaultAxesFontName', 'Segoe UI');
set(0, 'DefaultTextFontName', 'Segoe UI');

colors = {
    [0.8500 0.3250 0.0980], ... % M1: Red/Orange
    [0.9290 0.6940 0.1250], ... % M2: Yellow/Gold
    [0.0000 0.4470 0.7410], ... % M3: Blue
    [0.4660 0.6740 0.1880]      % M4: Green (Proposed S1+DHRL)
};

% -------------------------------------------------------------------------
% FIGURE 1: Trajectory Tracking Comparison (X and Z coordinates)
% -------------------------------------------------------------------------
fig1 = figure('Name', 'Trajectory Tracking Comparison', 'Color', 'w', 'Position', [100 100 1000 700], 'Visible', 'off');

subplot(2,1,1); hold on; grid on; box on;
plot(results.M1.t, results.M1.xref, 'k--', 'LineWidth', 2.0, 'DisplayName', 'Reference x_{ref}');
for i = 1:length(modelsKey)
    k = modelsKey{i};
    plot(results.(k).t, results.(k).xpd, 'Color', colors{i}, 'LineWidth', 1.5, 'DisplayName', results.(k).name);
end
ylabel('Position X [m]', 'FontSize', 12, 'FontWeight', 'bold');
title('Trajectory Tracking along X-Axis', 'FontSize', 14, 'FontWeight', 'bold');
legend('Location', 'best', 'FontSize', 10);

subplot(2,1,2); hold on; grid on; box on;
plot(results.M1.t, results.M1.zref, 'k--', 'LineWidth', 2.0, 'DisplayName', 'Reference z_{ref}');
for i = 1:length(modelsKey)
    k = modelsKey{i};
    plot(results.(k).t, results.(k).zpd, 'Color', colors{i}, 'LineWidth', 1.5, 'DisplayName', results.(k).name);
end
xlabel('Time [s]', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('Position Z [m]', 'FontSize', 12, 'FontWeight', 'bold');
title('Trajectory Tracking along Z-Axis', 'FontSize', 14, 'FontWeight', 'bold');
legend('Location', 'best', 'FontSize', 10);

saveas(fig1, fullfile(artifactDir, 'fig1_trajectory_tracking.png'));
saveas(fig1, fullfile(folder3D, 'fig1_trajectory_tracking.png'));
saveas(fig1, fullfile(rootDir, 'fig1_trajectory_tracking.png'));
close(fig1);
fprintf('Saved Figure 1: fig1_trajectory_tracking.png\n');

% -------------------------------------------------------------------------
% FIGURE 2: Tracking Errors Dynamics
% -------------------------------------------------------------------------
fig2 = figure('Name', 'Tracking Errors', 'Color', 'w', 'Position', [100 100 1000 700], 'Visible', 'off');

subplot(2,1,1); hold on; grid on; box on;
for i = 1:length(modelsKey)
    k = modelsKey{i};
    ex = results.(k).xref - results.(k).xpd;
    plot(results.(k).t, ex, 'Color', colors{i}, 'LineWidth', 1.5, 'DisplayName', results.(k).name);
end
ylabel('Error e_x(t) [m]', 'FontSize', 12, 'FontWeight', 'bold');
title('Tracking Error Dynamics e_x(t)', 'FontSize', 14, 'FontWeight', 'bold');
legend('Location', 'best', 'FontSize', 10);

subplot(2,1,2); hold on; grid on; box on;
for i = 1:length(modelsKey)
    k = modelsKey{i};
    ez = results.(k).zref - results.(k).zpd;
    plot(results.(k).t, ez, 'Color', colors{i}, 'LineWidth', 1.5, 'DisplayName', results.(k).name);
end
xlabel('Time [s]', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('Error e_z(t) [m]', 'FontSize', 12, 'FontWeight', 'bold');
title('Tracking Error Dynamics e_z(t)', 'FontSize', 14, 'FontWeight', 'bold');
legend('Location', 'best', 'FontSize', 10);

saveas(fig2, fullfile(artifactDir, 'fig2_tracking_errors.png'));
saveas(fig2, fullfile(folder3D, 'fig2_tracking_errors.png'));
saveas(fig2, fullfile(rootDir, 'fig2_tracking_errors.png'));
close(fig2);
fprintf('Saved Figure 2: fig2_tracking_errors.png\n');

% -------------------------------------------------------------------------
% FIGURE 3: Control Torques Comparison & Chattering Analysis
% -------------------------------------------------------------------------
fig3 = figure('Name', 'Control Signals & Chattering Analysis', 'Color', 'w', 'Position', [100 100 1100 800], 'Visible', 'off');

subplot(3,1,1); hold on; grid on; box on;
for i = 1:length(modelsKey)
    k = modelsKey{i};
    plot(results.(k).t, results.(k).u1, 'Color', colors{i}, 'LineWidth', 1.2, 'DisplayName', results.(k).name);
end
ylabel('Control Torque u_1 [N.m]', 'FontSize', 11, 'FontWeight', 'bold');
title('Control Inputs u_1(t) (Torque 1)', 'FontSize', 13, 'FontWeight', 'bold');
legend('Location', 'best', 'FontSize', 9);

subplot(3,1,2); hold on; grid on; box on;
for i = 1:length(modelsKey)
    k = modelsKey{i};
    plot(results.(k).t, results.(k).u2, 'Color', colors{i}, 'LineWidth', 1.2, 'DisplayName', results.(k).name);
end
ylabel('Control Torque u_2 [N.m]', 'FontSize', 11, 'FontWeight', 'bold');
title('Control Inputs u_2(t) (Torque 2)', 'FontSize', 13, 'FontWeight', 'bold');
legend('Location', 'best', 'FontSize', 9);

subplot(3,1,3); hold on; grid on; box on;
for i = 1:length(modelsKey)
    k = modelsKey{i};
    plot(results.(k).t, results.(k).u3, 'Color', colors{i}, 'LineWidth', 1.2, 'DisplayName', results.(k).name);
end
xlabel('Time [s]', 'FontSize', 11, 'FontWeight', 'bold');
ylabel('Control Torque u_3 [N.m]', 'FontSize', 11, 'FontWeight', 'bold');
title('Control Inputs u_3(t) (Torque 3)', 'FontSize', 13, 'FontWeight', 'bold');
legend('Location', 'best', 'FontSize', 9);

saveas(fig3, fullfile(artifactDir, 'fig3_control_torques.png'));
saveas(fig3, fullfile(folder3D, 'fig3_control_torques.png'));
saveas(fig3, fullfile(rootDir, 'fig3_control_torques.png'));
close(fig3);
fprintf('Saved Figure 3: fig3_control_torques.png\n');

% -------------------------------------------------------------------------
% FIGURE 4: Sliding Surfaces Dynamics \sigma(t)
% -------------------------------------------------------------------------
fig4 = figure('Name', 'Sliding Surface Dynamics', 'Color', 'w', 'Position', [100 100 1000 700], 'Visible', 'off');

subplot(2,1,1); hold on; grid on; box on;
for i = 1:length(modelsKey)
    k = modelsKey{i};
    plot(results.(k).t, results.(k).s1, 'Color', colors{i}, 'LineWidth', 1.5, 'DisplayName', results.(k).name);
end
ylabel('Sliding Surface s_1(t)', 'FontSize', 12, 'FontWeight', 'bold');
title('Sliding Surface Dynamics s_1(t) Convergence', 'FontSize', 14, 'FontWeight', 'bold');
legend('Location', 'best', 'FontSize', 10);

subplot(2,1,2); hold on; grid on; box on;
for i = 1:length(modelsKey)
    k = modelsKey{i};
    plot(results.(k).t, results.(k).s2, 'Color', colors{i}, 'LineWidth', 1.5, 'DisplayName', results.(k).name);
end
xlabel('Time [s]', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('Sliding Surface s_2(t)', 'FontSize', 12, 'FontWeight', 'bold');
title('Sliding Surface Dynamics s_2(t) Convergence', 'FontSize', 14, 'FontWeight', 'bold');
legend('Location', 'best', 'FontSize', 10);

saveas(fig4, fullfile(artifactDir, 'fig4_sliding_surfaces.png'));
saveas(fig4, fullfile(folder3D, 'fig4_sliding_surfaces.png'));
saveas(fig4, fullfile(rootDir, 'fig4_sliding_surfaces.png'));
close(fig4);
fprintf('Saved Figure 4: fig4_sliding_surfaces.png\n');

% -------------------------------------------------------------------------
% FIGURE 5: Quantitative Metric Comparisons (Bar Charts)
% -------------------------------------------------------------------------
fig5 = figure('Name', 'Performance Indices Comparison', 'Color', 'w', 'Position', [100 100 1100 700], 'Visible', 'off');

names = {results.M1.name, results.M2.name, results.M3.name, results.M4.name};
iae_vals = [metrics.M1.IAE, metrics.M2.IAE, metrics.M3.IAE, metrics.M4.IAE];
ise_vals = [metrics.M1.ISE, metrics.M2.ISE, metrics.M3.ISE, metrics.M4.ISE];
chat_vals = [metrics.M1.Chattering, metrics.M2.Chattering, metrics.M3.Chattering, metrics.M4.Chattering];
peak_u_vals = [metrics.M1.PeakControl, metrics.M2.PeakControl, metrics.M3.PeakControl, metrics.M4.PeakControl];

subplot(2,2,1);
b1 = bar(iae_vals, 'FaceColor', [0.2 0.6 0.8]); grid on;
set(gca, 'XTickLabel', {'M1: PD', 'M2: S2', 'M3: S1+Sat', 'M4: S1+DHRL'}, 'XTickLabelRotation', 15);
ylabel('IAE'); title('Integral of Absolute Error (IAE)', 'FontSize', 12, 'FontWeight', 'bold');

subplot(2,2,2);
b2 = bar(ise_vals, 'FaceColor', [0.8 0.4 0.4]); grid on;
set(gca, 'XTickLabel', {'M1: PD', 'M2: S2', 'M3: S1+Sat', 'M4: S1+DHRL'}, 'XTickLabelRotation', 15);
ylabel('ISE'); title('Integral of Squared Error (ISE)', 'FontSize', 12, 'FontWeight', 'bold');

subplot(2,2,3);
b3 = bar(chat_vals, 'FaceColor', [0.9 0.6 0.2]); grid on;
set(gca, 'XTickLabel', {'M1: PD', 'M2: S2', 'M3: S1+Sat', 'M4: S1+DHRL'}, 'XTickLabelRotation', 15);
ylabel('Total Variation'); title('Chattering Metric (Total Variation)', 'FontSize', 12, 'FontWeight', 'bold');

subplot(2,2,4);
b4 = bar(peak_u_vals, 'FaceColor', [0.4 0.7 0.4]); grid on;
set(gca, 'XTickLabel', {'M1: PD', 'M2: S2', 'M3: S1+Sat', 'M4: S1+DHRL'}, 'XTickLabelRotation', 15);
ylabel('Peak Control Torque [N.m]'); title('Peak Control Effort', 'FontSize', 12, 'FontWeight', 'bold');

saveas(fig5, fullfile(artifactDir, 'fig5_performance_metrics.png'));
saveas(fig5, fullfile(folder3D, 'fig5_performance_metrics.png'));
saveas(fig5, fullfile(rootDir, 'fig5_performance_metrics.png'));
close(fig5);
fprintf('Saved Figure 5: fig5_performance_metrics.png\n');

%% Save MAT and CSV files
save(fullfile(rootDir, 'simulation_results.mat'), 'results', 'metrics');
save(fullfile(artifactDir, 'simulation_results.mat'), 'results', 'metrics');
save(fullfile(folder3D, 'simulation_results.mat'), 'results', 'metrics');

% Export CSV
T = table(names', iae_vals', ise_vals', chat_vals', peak_u_vals', ...
    'VariableNames', {'Controller', 'IAE', 'ISE', 'ChatteringMetric', 'PeakControlEffort'});
writetable(T, fullfile(rootDir, 'simulation_results.csv'));
writetable(T, fullfile(artifactDir, 'simulation_results.csv'));
writetable(T, fullfile(folder3D, 'simulation_results.csv'));
fprintf('Exported results to simulation_results.csv and simulation_results.mat\n');

fprintf('\n=====================================================\n');
fprintf('  SIMULATION SUITE COMPLETED SUCCESSFULLY!\n');
fprintf('=====================================================\n');

%% Helper function to extract array from simulation outputs
function val = getVal(simOut, varName)
    try
        if isprop(simOut, varName) || isfield(simOut, varName)
            v = simOut.(varName);
        else
            v = evalin('base', varName);
        end
    catch
        try
            v = evalin('base', varName);
        catch
            error('Variable %s not found in simOut or base workspace', varName);
        end
    end
    
    if isa(v, 'timeseries')
        val = v.Data;
    elseif isstruct(v) && isfield(v, 'signals')
        val = v.signals.values;
    else
        val = double(v);
    end
    val = val(:);
end
