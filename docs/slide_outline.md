# Competition Presentation Deck (8 Slides)

## Slide 1: Title Slide
- **Title:** PID Controller Optimization via PSO Algorithm
- **Subtitle:** Automated Swarm-Based Gain Synthesis & Step-Response Validation in MATLAB/Simulink
- **Metadata:** May 2025 – Nov 2025 | Control Systems & Swarm Intelligence
- **Speaker Cue:** Introduce the core problem—PID controllers run over 90% of industrial loops, yet manual and classical tuning methods struggle to balance speed, stability, and zero steady-state error.

## Slide 2: Problem Statement & Limitations of Classical Tuning
- **Key Points:**
  - Trial-and-error tuning is slow and sensitive to plant dynamics.
  - **Ziegler–Nichols (ZN)** tuning pushes systems near marginal stability, producing high overshoot ($>40\%$) and prolonged oscillations.
  - **Objective:** Automate $[K_p, K_i, K_d]$ synthesis to simultaneously minimize overshoot ($M_p$), settling time ($t_s$), and steady-state error ($e_{ss}$).

## Slide 3: System Architecture & Benchmark Plant
- **Visual:** Screenshot of `simulink/pid_pso_demo.slx` block diagram.
- **Key Points:**
  - Third-order benchmark plant: $G(s) = \frac{1}{(s+1)(s+2)(s+3)} = \frac{1}{s^3 + 6s^2 + 11s + 6}$.
  - Closed-loop unity-feedback architecture with derivative filter ($N = 100$) for practical realizability.
  - Direct MATLAB-to-Simulink parameter injection (`Kp_pso`, `Ki_pso`, `Kd_pso`).

## Slide 4: Particle Swarm Optimization (PSO) Formulation
- **Key Points:**
  - **Search Space:** 3D continuous space $[K_p, K_i, K_d] \in [0, 45] \times [0, 45] \times [0, 25]$.
  - **Swarm Configuration:** 35 particles, 60 iterations, linearly decreasing inertia weight ($w: 0.9 \rightarrow 0.4$), $c_1 = c_2 = 1.5$.
  - **Velocity & Position Update:**
    $$v_i^{k+1} = w v_i^k + c_1 r_1 (p_{\text{best}, i} - x_i^k) + c_2 r_2 (g_{\text{best}} - x_i^k)$$

## Slide 5: Multi-Objective Fitness Function & Stability Enforcement
- **Key Points:**
  - **Cost Function:** $J = 1.0 \cdot \text{ITAE} + 0.6 \cdot M_p + 1.5 \cdot t_s + 25.0 \cdot e_{ss}$
  - **Stability Gate:** Any candidate gain vector producing right-half-plane closed-loop poles (`~isstable(T)`) receives an immediate penalty ($J = 10^6$), guaranteeing dynamic stability across all valid solutions.

## Slide 6: Optimization Convergence & Step Response Results
- **Visuals:** Place `results/convergence_curve.png` (left) and `results/step_comparison.png` (right).
- **Key Points:**
  - Rapid global cost convergence within the first 25–35 iterations.
  - **PSO-PID** eliminates aggressive overshoot while reaching steady-state reference faster than Ziegler–Nichols.

## Slide 7: Quantitative Benchmark Comparison
- **Visual:** Populate this table directly from your MATLAB Command Window printout after running `pso_pid_tuning.m`:

| Tuning Method | $K_p$ | $K_i$ | $K_d$ | Overshoot (%) | Settling Time (s) | Steady-State Error |
| :--- | :---: | :---: | :---: | :---: | :---: | :---: |
| **Ziegler–Nichols PID** | *from script* | *from script* | *from script* | *from script* | *from script* | *from script* |
| **PSO-Tuned PID** | *from script* | *from script* | *from script* | *from script* | *from script* | *from script* |

## Slide 8: Live Simulink Validation & Future Scope
- **Live Demo Action:** Switch to `simulink/pid_pso_demo.slx` and open `Comparison_Scope` to show real-time step tracking.
- **Future Extensions:** Hardware-in-the-loop (HIL) deployment on a DC motor / Arduino target and adaptive online PSO under load disturbances.