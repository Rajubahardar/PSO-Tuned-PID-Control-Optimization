function J = evaluate_pid(gains, G, t)
% EVALUATE_PID Multi-objective fitness function for PSO-PID tuning.
%   Minimizes ITAE, percent overshoot, settling time, and steady-state error.

    Kp = gains(1);
    Ki = gains(2);
    Kd = gains(3);

    % Construct PID controller with derivative filter (Tf = 0.01)
    C = pid(Kp, Ki, Kd, 0.01);
    T = feedback(C * G, 1);

    % Reject unstable closed-loop candidates immediately
    if ~isstable(T)
        J = 1e6;
        return;
    end

    y = step(T, t);
    if any(isnan(y)) || any(isinf(y))
        J = 1e6;
        return;
    end

    % Time-domain performance metrics
    e         = abs(1 - y);
    itae      = trapz(t, t(:) .* e(:));          % Integral of Time-weighted Absolute Error
    overshoot = max(0, (max(y) - 1) * 100);      % Percentage overshoot (%)
    sse       = abs(1 - y(end));                 % Steady-state error

    info = stepinfo(y, t, 1);
    ts   = info.SettlingTime;
    if isnan(ts)
        ts = t(end) * 2;
    end

    % Weighted multi-objective cost
    J = (1.0 * itae) + (0.6 * overshoot) + (1.5 * ts) + (25.0 * sse);
end