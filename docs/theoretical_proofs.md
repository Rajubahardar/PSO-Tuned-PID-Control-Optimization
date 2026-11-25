# Theoretical Proofs & Mathematical Formulation
**Project:** PID Controller Optimization via PSO Algorithm  
**Plant Model:** Third-Order Linear Time-Invariant (LTI) System  

---

## 1. Plant Transfer Function & Open-Loop Dynamics
The benchmark plant modeled in `src/pso_pid_tuning.m` and `simulink/pid_pso_demo.slx` is an overdamped third-order system with three distinct real poles in the open left-half $s$-plane ($s = -1, -2, -3$):

$$G(s) = \frac{1}{(s+1)(s+2)(s+3)} = \frac{1}{s^3 + 6s^2 + 11s + 6}$$

Because all open-loop poles lie strictly in the left-half plane ($\text{Re}(s) < 0$), $G(s)$ is bounded-input bounded-output (BIBO) stable with DC gain $G(0) = \frac{1}{6}$ and system **Type 0** (zero integrators). Without integral control, a unit-step reference yields a nonzero steady-state error.

---

## 2. Proof of Ziegler–Nichols Ultimate Gain ($K_u$) and Ultimate Period ($P_u$)
To establish the classical baseline via the **Ziegler–Nichols Closed-Loop (Ultimate Sensitivity) Method**, the controller is reduced to pure proportional gain $C(s) = K$. The closed-loop characteristic equation under unity negative feedback is:

$$1 + K G(s) = 0 \implies s^3 + 6s^2 + 11s + (6 + K) = 0$$

### Routh–Hurwitz Stability Array
Constructing the Routh table for the cubic characteristic polynomial:

$$\begin{array}{c|cc}
s^3 & 1 & 11 \\
s^2 & 6 & 6 + K \\
s^1 & b_1 & 0 \\
s^0 & c_1 & 0
\end{array}$$

Computing the first-column coefficients:

$$b_1 = \frac{(6)(11) - (1)(6 + K)}{6} = \frac{60 - K}{6}, \qquad c_1 = 6 + K$$

For all closed-loop poles to remain in the open left-half plane, every entry in the first column must be strictly positive:

$$6 + K > 0 \implies K > -6, \qquad \frac{60 - K}{6} > 0 \implies K < 60$$

Thus, the closed-loop proportional system is asymptotically stable for $-6 < K < 60$.

### Marginal Stability & Sustained Oscillation Frequency
Setting the $s^1$ row to zero yields the **ultimate gain** $K_u$ at which a pair of complex-conjugate poles crosses the imaginary axis ($j\omega$):

$$\frac{60 - K_u}{6} = 0 \implies K_u = 60$$

Forming the auxiliary polynomial $A(s)$ from the $s^2$ row at $K = K_u = 60$:

$$A(s) = 6s^2 + (6 + 60) = 6s^2 + 66 = 0 \implies s^2 = -11 \implies s = \pm j\sqrt{11}$$

Hence, the ultimate oscillation frequency is $\omega_u = \sqrt{11} \approx 3.3166\text{ rad/s}$, and the **ultimate period** is:

$$P_u = \frac{2\pi}{\omega_u} = \frac{2\pi}{\sqrt{11}} \approx 1.8944\text{ s}$$

### Classical Ziegler–Nichols PID Gains
Substituting $K_u = 60$ and $P_u = \frac{2\pi}{\sqrt{11}}$ into the standard Ziegler–Nichols PID rules gives the baseline gains computed in `src/pso_pid_tuning.m`:

$$K_{p,\text{ZN}} = 0.6 K_u = 36.000$$

$$K_{i,\text{ZN}} = \frac{K_{p,\text{ZN}}}{0.5 P_u} = \frac{36\sqrt{11}}{\pi} \approx 38.006$$

$$K_{d,\text{ZN}} = K_{p,\text{ZN}}(0.125 P_u) = \frac{9\pi}{\sqrt{11}} \approx 8.525$$

---

## 3. Realizable Filtered PID Transfer Function
An ideal parallel PID controller $C_{\text{ideal}}(s) = K_p + \frac{K_i}{s} + K_d s$ has numerator degree greater than denominator degree (improper), causing unbounded gain ($\lim_{\omega \to \infty} |C_{\text{ideal}}(j\omega)| = \infty$) that amplifies high-frequency measurement noise.

Both MATLAB (`pid(Kp, Ki, Kd, 0.01)`) and Simulink (`N = 100`) implement a **first-order low-pass derivative filter** with time constant $T_f = \frac{1}{N} = 0.01\text{ s}$:

$$C(s) = K_p + \frac{K_i}{s} + \frac{K_d s}{T_f s + 1} = \frac{(K_p T_f + K_d)s^2 + (K_p + K_i T_f)s + K_i}{s(T_f s + 1)}$$

This makes $C(s)$ **biproper** (second-order numerator over second-order denominator), bounding high-frequency gain to:

$$\lim_{\omega \to \infty} |C(j\omega)| = K_p + \frac{K_d}{T_f} = K_p + 100 K_d$$

---

## 4. Proofs of Zero Steady-State Error & Load Disturbance Rejection
Consider the two-input closed-loop system driven by reference setpoint $R(s)$ and plant-input load disturbance $D(s)$:

$$Y(s) = \underbrace{\frac{C(s)G(s)}{1 + C(s)G(s)}}_{T(s)} R(s) + \underbrace{\frac{G(s)}{1 + C(s)G(s)}}_{S_d(s)} D(s)$$

Write $C(s) = \frac{N_c(s)}{s(T_f s + 1)}$ with $N_c(0) = K_i > 0$, and $G(s) = \frac{1}{D_p(s)}$ with $D_p(0) = 6$.

### Case A: Unit-Step Reference Tracking ($R(s) = \frac{1}{s},\; D(s) = 0$)
The tracking error transfer function is $E(s) = R(s) - Y(s) = \frac{1}{1 + C(s)G(s)} R(s)$. Because $C(s)G(s)$ contains a single pole at $s = 0$, the open-loop system is **Type 1**. For any closed-loop stable gain vector ($K_i > 0$), the **Final Value Theorem (FVT)** yields:

$$e_{ss,\text{ref}} = \lim_{t \to \infty} e(t) = \lim_{s \to 0} s E(s) = \lim_{s \to 0} s \cdot \frac{s(T_f s + 1)D_p(s)}{s(T_f s + 1)D_p(s) + N_c(s)} \cdot \frac{1}{s} = \frac{0}{0 + K_i} = 0$$

### Case B: Step Load Disturbance Rejection ($R(s) = 0,\; D(s) = \frac{A_d}{s}$)
When a step load disturbance of amplitude $A_d = -0.3$ enters at the plant input ($t = 4\text{ s}$), the output deviation is:

$$Y_d(s) = \frac{G(s)}{1 + C(s)G(s)} \cdot \frac{A_d}{s} = \frac{A_d (T_f s + 1)}{s(T_f s + 1)D_p(s) + N_c(s)}$$

Applying the Final Value Theorem proves asymptotic disturbance rejection:

$$y_{d,ss} = \lim_{t \to \infty} y_d(t) = \lim_{s \to 0} s Y_d(s) = \lim_{s \to 0} \frac{s A_d (T_f s + 1)}{s(T_f s + 1)D_p(s) + N_c(s)} = \frac{0}{K_i} = 0$$

**Physical Interpretation:** The pole at $s = 0$ in $C(s)$ places a **blocking zero at $s = 0$** in the load sensitivity function $S_d(s) = \frac{G(s)}{1 + C(s)G(s)}$, guaranteeing complete steady-state recovery from constant load disturbances while the transient dip magnitude is governed by $|1 + C(j\omega)G(j\omega)|$.

---

## 5. $s$-Plane Pole Geometry & Frequency-Domain Stability Margins

### Dominant Closed-Loop Poles & Transient Response (`pole_zero_map.png`)
For a dominant complex-conjugate closed-loop pole pair $s_{1,2} = -\sigma \pm j\omega_d = -\zeta\omega_n \pm j\omega_n\sqrt{1 - \zeta^2}$:

$$M_p = 100 \exp\!\left(-\frac{\pi\zeta}{\sqrt{1 - \zeta^2}}\right), \qquad t_s\,(2\%) \approx \frac{4}{\sigma} = \frac{4}{\zeta\omega_n}, \qquad \zeta = \cos\theta$$

where $\theta$ is the angle between the pole vector and the negative real axis. Moving closed-loop poles farther left (larger $\sigma$) shortens settling time $t_s$, while reducing $\theta$ (higher damping ratio $\zeta$) suppresses overshoot $M_p$.

### Open-Loop Gain & Phase Margins (`margin(C * G)`)
Let $L(s) = C(s)G(s)$ denote the open-loop transfer function:
* **Gain Crossover Frequency ($\omega_{cp}$):** Frequency where $|L(j\omega_{cp})| = 1\;(0\text{ dB})$.
* **Phase Margin ($\text{PM}$):** $\text{PM} = 180^\circ + \angle L(j\omega_{cp})$.
* **Phase Crossover Frequency ($\omega_{cg}$):** Frequency where $\angle L(j\omega_{cg}) = -180^\circ$.
* **Gain Margin ($\text{GM}$):** $\text{GM}_{\text{dB}} = -20\log_{10}|L(j\omega_{cg})|$.

For second-order-dominated systems, damping ratio relates approximately to phase margin via $\zeta \approx \frac{\text{PM}}{100}$ ($\text{PM} < 70^\circ$). Classical Ziegler–Nichols tuning targets a quarter-decay ratio ($\zeta \approx 0.215$, $\text{PM} \approx 20^\circ\text{–}25^\circ$), causing pronounced oscillatory ringing. PSO synthesis increases both $\text{PM}$ (into the standard $45^\circ\text{–}65^\circ$ textbook design band) and $\text{GM}$ ($> 6\text{ dB}$).

---

## 6. Constrained Multi-Objective PSO Formulation
Each particle $i \in \{1, \dots, 35\}$ represents a candidate controller vector $\mathbf{x}_i = [K_p, K_i, K_d] \in \Omega$, where $\Omega = [0, 45] \times [0, 45] \times [0, 25]$.

### Fitness Function with Stability Barrier (`src/evaluate_pid.m`)
To enforce Hurwitz stability throughout the swarm search, unstable candidates are rejected via an exterior penalty barrier:

$$J(\mathbf{x}) = \begin{cases}
1.0\,\text{ITAE} + 0.6\,M_p + 1.5\,t_s + 25.0\,e_{ss}, & \text{if } \max_m \text{Re}(\lambda_m) < 0 \\
10^6, & \text{otherwise}
\end{cases}$$

where $\lambda_m$ are the closed-loop eigenvalues of $T(s)$ and $\text{ITAE} = \int_0^{T_{\text{sim}}} t\,|1 - y(t)|\,dt$ penalizes sustained late-time oscillations far more heavily than initial transient error.

### Swarm Kinematic Update Equations (`src/pso_pid_tuning.m`)
At iteration $k \in \{1, \dots, 60\}$, particle velocities and positions update according to:

$$\mathbf{v}_i^{k+1} = w(k)\mathbf{v}_i^k + c_1 \mathbf{r}_1 \odot (\mathbf{p}_{\text{best},i} - \mathbf{x}_i^k) + c_2 \mathbf{r}_2 \odot (\mathbf{g}_{\text{best}} - \mathbf{x}_i^k)$$

$$\mathbf{x}_i^{k+1} = \text{clip}\!\left(\mathbf{x}_i^k + \mathbf{v}_i^{k+1},\; \mathbf{x}_{\min},\; \mathbf{x}_{\max}\right)$$

with cognitive/social acceleration constants $c_1 = c_2 = 1.5$, uniform random vectors $\mathbf{r}_1, \mathbf{r}_2 \sim \mathcal{U}(0,1)^3$, and linearly annealed inertia weight:

$$w(k) = w_{\max} - \frac{w_{\max} - w_{\min}}{K_{\max}}\,k, \qquad w_{\max} = 0.9,\; w_{\min} = 0.4$$

High initial inertia ($w = 0.9$) promotes global exploration across $\Omega$, while lower terminal inertia ($w = 0.4$) refines local convergence around $\mathbf{g}_{\text{best}}$.