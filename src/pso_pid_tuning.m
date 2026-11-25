%% PSO-Tuned PID Control Optimization vs. Ziegler-Nichols
clear; clc; close all;
rng(42, 'twister'); % Fixed seed for reproducible competition results

% Resolve project directories automatically
srcDir     = fileparts(mfilename('fullpath'));
projectDir = fileparts(srcDir);
resultsDir = fullfile(projectDir, 'results');
if ~exist(resultsDir, 'dir'), mkdir(resultsDir); end
addpath(srcDir);

% 1. Define Benchmark 3rd-Order Plant: G(s) = 1 / ((s+1)(s+2)(s+3))
num = 1;
den = conv(conv([1 1], [1 2]), [1 3]); % s^3 + 6s^2 + 11s + 6
G   = tf(num, den);
t   = 0:0.01:8;

% 2. Classical Ziegler-Nichols Baseline (Ku = 60, Pu = 2*pi/sqrt(11))
Ku    = 60;
Pu    = 2 * pi / sqrt(11);
Kp_zn = 0.6 * Ku;
Ki_zn = Kp_zn / (0.5 * Pu);
Kd_zn = Kp_zn * (0.125 * Pu);

C_zn    = pid(Kp_zn, Ki_zn, Kd_zn, 0.01);
T_zn    = feedback(C_zn * G, 1);
y_zn    = step(T_zn, t);
info_zn = stepinfo(y_zn, t, 1);

% 3. PSO Swarm Configuration [Kp, Ki, Kd]
nParticles = 35;
maxIter    = 60;
wMax       = 0.9;   % Linearly decreasing inertia weight
wMin       = 0.4;
c1         = 1.5;   % Cognitive acceleration
c2         = 1.5;   % Social acceleration

lb = [0,  0,  0];   % Lower gain bounds
ub = [45, 45, 25];  % Upper gain bounds

pos         = lb + rand(nParticles, 3) .* (ub - lb);
vel         = zeros(nParticles, 3);
pBest       = pos;
pBestCost   = inf(nParticles, 1);
gBest       = pos(1, :);
gBestCost   = inf;
costHistory = zeros(maxIter, 1);

% 4. PSO Optimization Loop
fprintf('Running PSO-PID Optimization (%d particles, %d iterations)...\n', nParticles, maxIter);
for iter = 1:maxIter
    w = wMax - ((wMax - wMin) * (iter / maxIter));

    for i = 1:nParticles
        cost = evaluate_pid(pos(i, :), G, t);
        if cost < pBestCost(i)
            pBestCost(i) = cost;
            pBest(i, :)  = pos(i, :);
        end
        if cost < gBestCost
            gBestCost = cost;
            gBest     = pos(i, :);
        end
    end
    costHistory(iter) = gBestCost;

    r1  = rand(nParticles, 3);
    r2  = rand(nParticles, 3);
    vel = w * vel + c1 * r1 .* (pBest - pos) + c2 * r2 .* (gBest - pos);
    pos = max(lb, min(ub, pos + vel));
end

% 5. Evaluate Optimal PSO-Tuned PID
Kp_pso   = gBest(1);
Ki_pso   = gBest(2);
Kd_pso   = gBest(3);
C_pso    = pid(Kp_pso, Ki_pso, Kd_pso, 0.01);
T_pso    = feedback(C_pso * G, 1);
y_pso    = step(T_pso, t);
info_pso = stepinfo(y_pso, t, 1);

% 6. Save Tuned Gains for Simulink & Reporting
save(fullfile(resultsDir, 'tuned_gains.mat'), ...
    'Kp_pso', 'Ki_pso', 'Kd_pso', 'Kp_zn', 'Ki_zn', 'Kd_zn', ...
    'num', 'den', 'info_pso', 'info_zn', 'costHistory');

% 7. Print Competition Benchmark Table
fprintf('\n==================== PERFORMANCE COMPARISON ====================\n');
fprintf('%-16s | %7s | %7s | %7s | %11s | %11s | %9s\n', ...
    'Method', 'Kp', 'Ki', 'Kd', 'Overshoot %', 'Settling(s)', 'SS Error');
fprintf('----------------------------------------------------------------\n');
fprintf('%-16s | %7.3f | %7.3f | %7.3f | %11.2f | %11.2f | %9.4f\n', ...
    'Ziegler-Nichols', Kp_zn, Ki_zn, Kd_zn, info_zn.Overshoot, info_zn.SettlingTime, abs(1 - y_zn(end)));
fprintf('%-16s | %7.3f | %7.3f | %7.3f | %11.2f | %11.2f | %9.4f\n', ...
    'PSO-Tuned PID', Kp_pso, Ki_pso, Kd_pso, info_pso.Overshoot, info_pso.SettlingTime, abs(1 - y_pso(end)));
fprintf('================================================================\n');

% 8. Export High-Resolution Plot 1: Step Response Comparison
fig1 = figure('Color', 'w', 'Position', [100, 100, 720, 440]);
plot(t, y_zn, '--r', 'LineWidth', 1.8); hold on;
plot(t, y_pso, '-b', 'LineWidth', 2.4);
yline(1, ':k', 'Target Setpoint', 'LabelHorizontalAlignment', 'left');
title('Closed-Loop Step Response: Ziegler-Nichols vs. PSO-Tuned PID', 'FontSize', 12);
xlabel('Time (s)', 'FontSize', 11);
ylabel('System Output y(t)', 'FontSize', 11);
legend('Ziegler-Nichols PID', 'PSO-Tuned PID', 'Location', 'SouthEast');
grid on;
exportgraphics(fig1, fullfile(resultsDir, 'step_comparison.png'), 'Resolution', 300);

% 9. Export High-Resolution Plot 2: PSO Convergence Curve
fig2 = figure('Color', 'w', 'Position', [840, 100, 720, 440]);
plot(1:maxIter, costHistory, '-o', 'Color', [0.1 0.5 0.2], 'LineWidth', 2, 'MarkerSize', 4);
title('PSO Multi-Objective Fitness Convergence', 'FontSize', 12);
xlabel('Iteration', 'FontSize', 11);
ylabel('Global Best Cost J', 'FontSize', 11);
grid on;
exportgraphics(fig2, fullfile(resultsDir, 'convergence_curve.png'), 'Resolution', 300);

fprintf('Saved plots and tuned_gains.mat to: %s\n', resultsDir);