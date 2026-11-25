# Textbook Curriculum Alignment & Method Comparison
**Project:** PID Controller Optimization via PSO Algorithm  

---

## 1. Alignment with Standard Control & Optimization Textbooks

| Project Module / Feature | Textbook Concept | Mathematical Basis | Primary Textbook Reference |
| :--- | :--- | :--- | :--- |
| `G = tf(1, [1 6 11 6])` | Higher-Order LTI System Modeling | Continuous-time transfer function $G(s) = \frac{Y(s)}{U(s)}$ with real negative poles | **Ogata**, *Modern Control Engineering* (5th ed.), Ch. 2 & Ch. 5; **Nise**, *Control Systems Engineering* (8th ed.), Ch. 4 |
| `Ku = 60`, `Pu = 2*pi/sqrt(11)` | Routh–Hurwitz Criterion & Ultimate Sensitivity Tuning | Marginal stability boundary on the $j\omega$-axis; quarter-decay-ratio PID synthesis | **Ogata**, Ch. 6 (*Routh's Stability Criterion*) & Ch. 8 (*Ziegler–Nichols Rules*) |
| `pid(Kp, Ki, Kd, 0.01)` | Realizable PID with Derivative Filtering | First-order low-pass filter $T_f = 1/N$ preventing high-frequency noise amplification | **Åström & Hägglund**, *PID Controllers: Theory, Design, and Tuning* (2nd ed.), Ch. 3 |
| `stepinfo(y, t, 1)` | Time-Domain Transient Specifications | Percent overshoot ($M_p$), 2% settling time ($t_s$), and steady-state error ($e_{ss}$) | **Dorf & Bishop**, *Modern Control Systems* (13th ed.), Ch. 5; **Nise**, Ch. 4 |
| `trapz(t, t .* abs(1 - y))` | Optimal Control Performance Indices | Integral of Time-weighted Absolute Error ($\text{ITAE} = \int_0^T t|e(t)|\,dt$) | **Dorf & Bishop**, Ch. 5 (*Performance Indices*); **Ogata**, Ch. 8 |
| `margin(C * G)` | Bode / Nyquist Relative Stability Margins | Gain Margin ($\text{GM}_{\text{dB}}$) and Phase Margin ($\text{PM}^\circ$) at crossover frequencies | **Ogata**, Ch. 9 (*Frequency-Response Analysis*); **Nise**, Ch. 10 |
| `pzplot(T_zn, T_pso)` | Closed-Loop Root Locus & $s$-Plane Stability | Left-half-plane pole migration, damping ratio $\zeta = \cos\theta$, and natural frequency $\omega_n$ | **Nise**, Ch. 8 (*Root Locus Techniques*); **Dorf & Bishop**, Ch. 7 |
| `Load_Disturbance` at $t = 4\text{ s}$ | Sensitivity & Disturbance Rejection | Load sensitivity $S_d(s) = \frac{G(s)}{1 + C(s)G(s)}$ and Type-1 disturbance rejection via FVT | **Åström & Murray**, *Feedback Systems*, Ch. 11 (*Sensitivity Functions*) |
| `pso_pid_tuning.m` | Swarm Intelligence Metaheuristic Optimization | Velocity-position swarm dynamics with linearly decreasing inertia weight | **Kennedy & Eberhart** (1995), *IEEE ICNN*; **Z. L. Gaing** (2004), *IEEE Trans. Energy Conversion* |

---

## 2. Theoretical Comparison: Classical Tuning vs. PSO-PID

| Evaluation Dimension | Ziegler–Nichols (Closed-Loop) | Analytical Pole Placement / Root Locus | PSO-Tuned PID (This Project) |
| :--- | :--- | :--- | :--- |
| **Design Philosophy** | Empirical rules derived from ultimate gain $K_u$ and period $P_u$ | Exact algebraic assignment of closed-loop characteristic polynomial roots | Global stochastic optimization over a weighted multi-objective cost landscape |
| **Mathematical Requirement** | Requires bringing physical or simulated plant to marginal stability | Requires exact symbolic plant order and zero-pole cancellation awareness | Requires only black-box step-response simulation (`step(T, t)`) |
| **Transient Overshoot ($M_p$)** | High (typically $40\%\text{–}60\%$ due to quarter-decay-ratio design) | Exact for 2nd-order plants; degraded by unmodeled higher-order poles/zeros | Directly minimized via explicit $M_p$ and $\text{ITAE}$ penalty terms |
| **Stability Margins ($\text{GM}, \text{PM}$)** | Narrow ($\text{PM} \approx 15^\circ\text{–}25^\circ$), prone to oscillatory ringing | Specified indirectly via desired damping ratio $\zeta$ | High ($\text{PM} \approx 45^\circ\text{–}65^\circ$, $\text{GM} > 6\text{ dB}$), well-damped response |
| **Disturbance Rejection** | Fast initial recovery but prolonged post-disturbance oscillation | Trade-off between observer/controller bandwidth and noise sensitivity | Fast recovery with minimal transient drop and zero steady-state offset |
| **Computational Cost** | $O(1)$ closed-form algebraic formulas | $O(1)$ algebraic coefficient matching | Iterative ($N_{\text{particles}} \times N_{\text{iter}} = 2{,}100$ step evaluations, ~2–4 s in MATLAB) |

---

## 3. Viva & Competition Q&A Defense Guide

1. **Why does Ziegler–Nichols produce large overshoot on $G(s) = \frac{1}{(s+1)(s+2)(s+3)}$?**  
   Ziegler–Nichols rules were empirically calibrated for a **quarter-amplitude decay ratio** ($e^{-2\pi\zeta/\sqrt{1-\zeta^2}} = 0.25 \implies \zeta \approx 0.215$). Under a standard second-order approximation, $\zeta = 0.215$ inherently produces $M_p \approx 50\%$ overshoot and a narrow phase margin ($\approx 20^\circ$).

2. **Why choose ITAE over ISE (Integral of Squared Error) or IAE?**  
   In a step response, the initial error at $t = 0$ is always $e(0) = 1$ regardless of controller gains. **ISE** ($\int e^2\,dt$) heavily penalizes this unavoidable initial error, forcing aggressive high-gain controllers that cause large overshoot. **ITAE** ($\int t|e(t)|\,dt$) weights errors by elapsed time $t$, ignoring the unavoidable initial error while strictly penalizing lingering oscillations and slow settling.

3. **How does the algorithm guarantee that PSO never selects destabilizing PID gains?**  
   Inside `src/evaluate_pid.m`, every candidate $[K_p, K_i, K_d]$ is tested via `isstable(T)` (which checks whether all closed-loop poles have strictly negative real parts). Any unstable candidate is immediately assigned $J = 10^6$, steering particles away from the Routh–Hurwitz stability boundary.

4. **Why is a derivative filter ($N = 100,\; T_f = 0.01\text{ s}$) included in both MATLAB and Simulink?**  
   Pure differentiation ($K_d s$) is non-causal/improper and amplifies high-frequency sensor noise unboundedly. Adding a first-order pole at $s = -1/T_f = -100\text{ rad/s}$ preserves derivative phase lead near the crossover frequency ($\omega_{cp} \ll 100\text{ rad/s}$) while rolling off high-frequency gain.