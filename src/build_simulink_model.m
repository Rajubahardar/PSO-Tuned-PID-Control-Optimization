%% Auto-Build Side-by-Side Simulink Demo Model (pid_pso_demo.slx)
clearvars -except Kp_pso Ki_pso Kd_pso Kp_zn Ki_zn Kd_zn num den;
clc;

srcDir      = fileparts(mfilename('fullpath'));
projectDir  = fileparts(srcDir);
simulinkDir = fullfile(projectDir, 'simulink');
resultsDir  = fullfile(projectDir, 'results');
if ~exist(simulinkDir, 'dir'), mkdir(simulinkDir); end

% Load tuned gains if not already in workspace
gainsFile = fullfile(resultsDir, 'tuned_gains.mat');
if ~exist('Kp_pso', 'var')
    if exist(gainsFile, 'file')
        load(gainsFile);
    else
        error('Run pso_pid_tuning.m first to generate tuned_gains.mat.');
    end
end

modelName = 'pid_pso_demo';
modelPath = fullfile(simulinkDir, [modelName '.slx']);

% Close and overwrite if model already exists
if bdIsLoaded(modelName)
    close_system(modelName, 0);
end
if exist(modelPath, 'file')
    delete(modelPath);
end

new_system(modelName);
open_system(modelName);
set_param(modelName, 'StopTime', '8', 'Solver', 'ode45');

% --- Shared Step Input & Comparison Scope ---
add_block('simulink/Sources/Step', [modelName '/Step_Ref'], ...
    'Position', [40, 145, 80, 185], 'Time', '0', 'Before', '0', 'After', '1');

add_block('simulink/Signal Routing/Mux', [modelName '/Mux'], ...
    'Position', [580, 130, 585, 220], 'Inputs', '3');

add_block('simulink/Sinks/Scope', [modelName '/Comparison_Scope'], ...
    'Position', [640, 155, 680, 195]);

% --- Branch 1: PSO-Tuned PID Loop (Top) ---
add_block('simulink/Math Operations/Sum', [modelName '/Sum_PSO'], ...
    'Position', [160, 75, 190, 105], 'Inputs', '|+-');

add_block('simulink/Continuous/PID Controller', [modelName '/PSO_PID'], ...
    'Position', [255, 68, 315, 112], ...
    'P', 'Kp_pso', 'I', 'Ki_pso', 'D', 'Kd_pso', 'N', '100');

add_block('simulink/Continuous/Transfer Fcn', [modelName '/Plant_PSO'], ...
    'Position', [385, 68, 485, 112], ...
    'Numerator', mat2str(num), 'Denominator', mat2str(den));

% --- Branch 2: Ziegler-Nichols PID Loop (Bottom) ---
add_block('simulink/Math Operations/Sum', [modelName '/Sum_ZN'], ...
    'Position', [160, 225, 190, 255], 'Inputs', '|+-');

add_block('simulink/Continuous/PID Controller', [modelName '/ZN_PID'], ...
    'Position', [255, 218, 315, 262], ...
    'P', 'Kp_zn', 'I', 'Ki_zn', 'D', 'Kd_zn', 'N', '100');

add_block('simulink/Continuous/Transfer Fcn', [modelName '/Plant_ZN'], ...
    'Position', [385, 218, 485, 262], ...
    'Numerator', mat2str(num), 'Denominator', mat2str(den));

% --- Connect Signals with Auto-Routing ---
% Reference connections
add_line(modelName, 'Step_Ref/1', 'Sum_PSO/1', 'autorouting', 'on');
add_line(modelName, 'Step_Ref/1', 'Sum_ZN/1',  'autorouting', 'on');
add_line(modelName, 'Step_Ref/1', 'Mux/1',     'autorouting', 'on');

% PSO loop connections
add_line(modelName, 'Sum_PSO/1',   'PSO_PID/1',   'autorouting', 'on');
add_line(modelName, 'PSO_PID/1',   'Plant_PSO/1', 'autorouting', 'on');
add_line(modelName, 'Plant_PSO/1', 'Sum_PSO/2',   'autorouting', 'on');
add_line(modelName, 'Plant_PSO/1', 'Mux/2',       'autorouting', 'on');

% Ziegler-Nichols loop connections
add_line(modelName, 'Sum_ZN/1',   'ZN_PID/1',   'autorouting', 'on');
add_line(modelName, 'ZN_PID/1',   'Plant_ZN/1', 'autorouting', 'on');
add_line(modelName, 'Plant_ZN/1', 'Sum_ZN/2',   'autorouting', 'on');
add_line(modelName, 'Plant_ZN/1', 'Mux/3',      'autorouting', 'on');

% Scope connection
add_line(modelName, 'Mux/1', 'Comparison_Scope/1', 'autorouting', 'on');

% Save & Run Simulation for Instant Demo Readiness
save_system(modelName, modelPath);
sim(modelName);
open_system([modelName '/Comparison_Scope']);
fprintf('Simulink model built, saved, and simulated at:\n  %s\n', modelPath);