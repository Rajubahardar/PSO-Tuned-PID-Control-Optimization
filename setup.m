%% One-Click Environment Setup: PSO-Tuned PID Control Optimization
rootDir = fileparts(mfilename('fullpath'));
addpath(fullfile(rootDir, 'src'));
gainsFile = fullfile(rootDir, 'results', 'tuned_gains.mat');
if exist(gainsFile, 'file')
    load(gainsFile);
    fprintf('Loaded tuned gains: Kp = %.3f, Ki = %.3f, Kd = %.3f\n', Kp_pso, Ki_pso, Kd_pso);
else
    run(fullfile(rootDir, 'src', 'pso_pid_tuning.m'));
end
open_system(fullfile(rootDir, 'simulink', 'pid_pso_demo.slx'));
