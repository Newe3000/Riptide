# Riptide — Control Theory Notes

Dynamic model, LQR, MPC, and stability proofs for the Riptide test rig
(floating-base AUV + 7-DoF Franka arm, hull thrusters, end-effector
stabilization under disturbances).

These notes are written to PhD level: assumptions are stated explicitly, results
are proved, and every abstract object is tied back to what the code actually
computes (Pinocchio `M, C, g`; the MuJoCo fluid model; the thruster allocation
matrix `A`; the implemented impedance law). Where the rig departs from the theory
(model–plant mismatch, underactuation, two interacting controllers) this is
flagged rather than hidden — those gaps are exactly the interesting research
questions.

**Notation.** Vectors are column vectors; $I_n$ is the $n\times n$ identity;
$A\succ 0$ ($\succeq 0$) means symmetric positive (semi)definite; $\lambda_{\min}(\cdot)$
is the smallest eigenvalue; $\hat{a}$ is the skew-symmetric matrix with
$\hat a\,b = a\times b$; $\mathrm{SO}(3)$, $\mathrm{SE}(3)$ the rotation and rigid-motion groups.

---

## 1. Plant model

### 1.1 Configuration and velocity

The plant is an **underwater vehicle–manipulator system (UVMS)**: a free-floating
6-DoF base carrying an $n=7$-DoF serial arm.

- Base pose $\;\eta_b=(p_b, R_b)\in\mathrm{SE}(3)$: position $p_b\in\mathbb R^3$,
  orientation $R_b\in\mathrm{SO}(3)$ (the code stores $R_b$ as a unit quaternion).
- Arm joints $q\in\mathbb R^{n}$.

Because $\mathrm{SO}(3)$ is not a vector space we do **not** differentiate the pose
directly; we use the **generalized velocity**

$$
\nu \;=\; \begin{bmatrix} v_b \\ \omega_b \\ \dot q \end{bmatrix}\in\mathbb R^{6+n},
\qquad
v_b\in\mathbb R^3 \text{ (linear)},\;\; \omega_b\in\mathbb R^3 \text{ (angular)},
$$

with the **kinematic reconstruction** (the map from body twist to pose rate)

$$
\dot p_b = R_b v_b,\qquad \dot R_b = R_b\,\hat\omega_b,\qquad \dot q = \dot q .
\tag{1}
$$

(In Fossen's marine convention $v_b,\omega_b$ are **body-frame**; the free-joint
sensor in `mujoco_system.cpp` reports linear velocity in world and angular in
body — a frame detail the controllers must respect, and one place model errors
creep in.)

### 1.2 Equations of motion (UVMS)

Applying Lagrangian / Newton–Euler mechanics to the coupled base+arm gives the
standard **floating-base manipulator dynamics** in Fossen/Antonelli form
[Fossen 2011; Antonelli 2014; Schjølberg & Fossen 1994; Featherstone 2008]:

$$
\boxed{\;M(q)\,\dot\nu \;+\; C(q,\nu)\,\nu \;+\; D(q,\nu)\,\nu \;+\; g(q,R_b)
\;=\; \tau_{\mathrm{act}} \;+\; \tau_{\mathrm{ext}}\;}
\tag{2}
$$

term by term, with $N:=6+n$:

- $M(q)=M_{\mathrm{RB}}(q)+M_{A}(q)\in\mathbb R^{N\times N}$, $M=M^\top\succ0$: the
  **generalized inertia**. $M_{\mathrm{RB}}$ is the rigid-body mass matrix (what
  Pinocchio's `crba` returns for the arm block); $M_A$ is the **added mass** from
  accelerating the surrounding fluid (MuJoCo's ellipsoid `fluidshape` model
  contributes this on the hull).
- $C(q,\nu)\,\nu$: **Coriolis/centripetal** generalized forces (Pinocchio's
  `nonLinearEffects` = $C\nu+g$ for the arm). $C$ can be chosen so that
  $\dot M-2C$ is skew-symmetric — a property we exploit in §2.
- $D(q,\nu)\,\nu$: **hydrodynamic damping** (drag). Typically
  $D(\nu)=D_{\text{lin}}+D_{\text{quad}}(\nu)$ with a quadratic (Morison) term
  $\propto |\nu|\nu$. In the sim this is the velocity-dependent part of the
  ellipsoid fluid model. $D\succeq 0$ (dissipative).
- $g(q,R_b)$: **restoring** generalized force (gravity − buoyancy). The rig models
  **neutral buoyancy** as $g\equiv 0$ (`gravity="0 0 0"` in the MJCF), because
  MuJoCo's fluid model supplies no Archimedes term. A higher-fidelity model would
  use $g(\eta)=-[\,R^\top(W-B)\hat z;\; r_g\times R^\top W\hat z - r_b\times R^\top B\hat z\,]$
  (Fossen §4).
- $\tau_{\mathrm{ext}}$: **external disturbance** wrench (the current/impulse the
  `disturbance_generator` publishes; applied to `xfrc_applied` in the sim).

### 1.3 Actuation and control allocation

The system is **not fully actuated at the base**. The generalized force splits as

$$
\tau_{\mathrm{act}}
=\underbrace{\begin{bmatrix} B_{\mathrm{thr}}(q)\, u \\[2pt] 0\end{bmatrix}}_{\text{thrusters on the base}}
\;+\;
\underbrace{\begin{bmatrix} 0 \\[2pt] \tau_{\mathrm{arm}}\end{bmatrix}}_{\text{joint torques}} ,
\tag{3}
$$

where $u\in\mathbb R^{m}$ ($m=5$) are the individual thruster forces and
$\tau_{\mathrm{arm}}\in\mathbb R^{7}$ the joint torques (the effort commands over the
`ros2_control` seam). The arm is **fully actuated** ($7$ torques, $7$ DoF).

**Thruster allocation.** A thruster $i$ with unit force axis $a_i$ applied at body
point $r_i$ produces the body wrench $\big[a_i;\, r_i\times a_i\big]$ per unit
thrust. Stacking columns gives the **allocation (configuration) matrix**

$$
W_{\text{body}} \;=\; T\,u,\qquad
T=\begin{bmatrix} a_1 & \cdots & a_m \\ r_1\times a_1 & \cdots & r_m\times a_m\end{bmatrix}\in\mathbb R^{6\times m}.
\tag{4}
$$

The rig actuates only $\{F_x,F_z,M_x,M_y\}$ (surge, heave, roll, pitch); rows for
sway $F_y$ and yaw $M_z$ are dropped, giving $A\in\mathbb R^{4\times m}$ (the matrix
`BaseThrusterController` builds from config geometry). Given a desired wrench
$W_d\in\mathbb R^4$, the **minimum-energy allocation** is the (regularized)
pseudo-inverse

$$
u^\star=\arg\min_u \tfrac12\|u\|^2 \ \text{ s.t. } Au=W_d
\;\;\Longrightarrow\;\;
u^\star = A^\top\big(AA^\top+\epsilon I\big)^{-1}W_d ,
\tag{5}
$$

followed by a saturation $u_i\in[-\bar u,\bar u]$ [Fossen & Johansen 2006 survey
control allocation; the $\epsilon I$ is Tikhonov regularization for robustness near
rank deficiency]. Sway/yaw are **structurally uncontrollable** with this thruster
set — a fact that reappears in the stabilizability analysis of §3.4.

### 1.4 Task (operational) space

Let $x=k(q,\eta_b)\in\mathbb R^6$ be the end-effector pose (position + an
orientation parameterization; the code uses the axis–angle of the rotation error).
Differentiating,

$$
\dot x = J(q,\eta_b)\,\nu,\qquad J=\big[\,\underbrace{J_b}_{6\times6}\ \big|\ \underbrace{J_a}_{6\times n}\,\big],
\tag{6}
$$

the **geometric Jacobian**. Pinocchio's `getFrameJacobian(LOCAL_WORLD_ALIGNED)`
returns $J_a$; the code composes it with the measured base pose to form the
world-frame EE Jacobian (`PinocchioModel::jacobian`). Projecting (2) into task
space (Khatib's **operational-space formulation** [Khatib 1987]) yields, for the
fully-actuated arm with $J:=J_a$ full row rank,

$$
\Lambda(q)\,\ddot x + \mu(q,\nu)\,\dot x + p(q) = F,\qquad
\Lambda=(J M^{-1} J^\top)^{-1},\quad \tau_{\mathrm{arm}}=J^\top F ,
\tag{7}
$$

with $\Lambda\succ0$ the **operational-space inertia**, $\mu$ the task-space
Coriolis, $p$ the task-space gravity. Equation (7) is the model the EE controller
acts on.

### 1.5 What the *plant* is vs. what the *controller* knows

A point worth internalizing before trusting any proof below:

| Effect | Sim (plant, MuJoCo) | Controller model (Pinocchio) |
|---|---|---|
| Arm rigid-body $M,C,g$ | yes | yes |
| Added mass $M_A$ | yes (hull, ellipsoid fluid) | **no** |
| Drag $D(\nu)\nu$ | yes | **no** |
| Buoyancy/gravity | $g=0$ | $g=0$ |
| Base–arm inertial coupling | yes (full) | **partly** (EE pose composed with base; base treated as measured, not modeled) |

So every stability statement below is for the **nominal model**; on the real plant
it becomes a **robustness** question (model–plant mismatch $\Delta$). The honest
framing: our controllers are *certainty-equivalence* designs, and the drag $D\succeq0$
actually *helps* (it is passive/dissipative), while the unmodeled coupling is the
adversary. This is the natural entry point for robust/adaptive extensions.

---

## 2. Baseline: task-space impedance (implemented) and its Lyapunov stability

The currently-implemented law (`TaskSpaceImpedance`) is

$$
F = K_p\,\tilde x - K_d\,\dot x_{\text{arm}} + \text{(nullspace posture)},\qquad
\tilde x := x_d - x,\quad \dot x_{\text{arm}} = J_a\dot q,
\tag{8}
$$

with $\tau_{\mathrm{arm}}=J^\top F + \big(I-J^\top \bar J^\top\big)\tau_0$ and
gravity/Coriolis compensated by adding $\hat C\nu+\hat g$ [Hogan 1985; Ortega &
Spong 1989]. This is included here because (i) it is what runs, and (ii) its proof
is the cleanest illustration of the energy/passivity argument that LQR and MPC then
generalize.

**Proposition 2.1 (impedance stability, regulation).** Consider the nominal
task-space model (7) with exact gravity/Coriolis compensation and the control (8)
with $\dot x_d=0$, $K_p=K_p^\top\succ0$, $K_d=K_d^\top\succ0$, and $J$ nonsingular
on the trajectory. Then $x_d$ is an asymptotically stable equilibrium of the
closed loop.

*Proof.* With compensation, (7) under (8) becomes
$\Lambda\ddot x + K_d\dot x + K_p(x-x_d)=0$ (writing $\dot x$ for $\dot x_{\text{arm}}$
at $\dot x_d=0$). Take the total energy

$$
V(\tilde x,\dot x)=\tfrac12\,\dot x^\top \Lambda\,\dot x + \tfrac12\,\tilde x^\top K_p\,\tilde x \;\ge 0,
$$

$V=0\iff(\tilde x,\dot x)=0$. Differentiate along the closed loop, using
$\dot{\tilde x}=-\dot x$:

$$
\dot V=\dot x^\top\Lambda\ddot x+\tfrac12\dot x^\top\dot\Lambda\,\dot x-\dot x^\top K_p\tilde x .
$$

Substitute $\Lambda\ddot x=-\mu\dot x-K_d\dot x-K_p(x-x_d)=-\mu\dot x-K_d\dot x+K_p\tilde x$
from the closed-loop dynamics, and use the skew-symmetry of $\dot\Lambda-2\mu$
(so $\tfrac12\dot x^\top\dot\Lambda\,\dot x=\dot x^\top\mu\dot x$). The $\mu$ terms
cancel and the $\pm\dot x^\top K_p\tilde x$ terms cancel, leaving

$$
\dot V = -\,\dot x^\top K_d\,\dot x \;\le\; 0 .
$$

Thus $V$ is non-increasing; $\dot V=0\Rightarrow\dot x=0\Rightarrow\ddot x=0\Rightarrow
K_p\tilde x=0\Rightarrow\tilde x=0$. By LaSalle's invariance principle the largest
invariant set in $\{\dot V=0\}$ is $\{(\tilde x,\dot x)=0\}$, giving asymptotic
stability. $\qquad\blacksquare$

Two remarks that motivate LQR/MPC. (a) The gains $K_p,K_d$ are chosen by hand;
LQR chooses them **optimally** for a quadratic cost. (b) Nothing here respects
**constraints** (torque/joint limits); MPC adds exactly that.

---

## 3. Linear–Quadratic Regulator (LQR)

### 3.1 From nonlinear robot to a linear design model

Two standard routes produce a linear model $\dot z=Az+Bw$ on which LQR is designed.

**(a) Jacobian linearization.** About an equilibrium $(\bar\zeta,\bar\tau)$ with
$g=0$ this gives $\delta\dot z=A\,\delta z+B\,\delta\tau$ with
$A=\partial f/\partial z$, $B=\partial f/\partial \tau$; LQR then yields a **locally**
valid controller (valid in a neighborhood; the nonlinearity is the modeling error).

**(b) Feedback linearization (computed torque).** Preferred for the fully-actuated
arm. Choose $\tau_{\mathrm{arm}}=J^\top\!\big(\Lambda\,w+\mu\dot x+p\big)$ so that (7)
becomes the **double integrator**

$$
\ddot x = w .
\tag{9}
$$

Define the task error state $z=\big[\,e;\ \dot e\,\big]$, $e:=x-x_d$. Then

$$
\dot z = A z + B w,\qquad
A=\begin{bmatrix}0 & I_6\\ 0 & 0\end{bmatrix},\quad
B=\begin{bmatrix}0\\ I_6\end{bmatrix}.
\tag{10}
$$

$(A,B)$ is controllable (Kalman rank $=12$), so an LQR exists.

### 3.2 The LQR problem and its solution

Minimize the infinite-horizon quadratic cost [Kalman 1960; Anderson & Moore 1990;
Lewis, Vrabie & Syrmos 2012]

$$
J=\int_0^\infty\!\big(z^\top Q\,z + w^\top R\,w\big)\,dt,\qquad Q=Q^\top\succeq0,\ R=R^\top\succ0 .
\tag{11}
$$

**Theorem 3.1 (LQR).** If $(A,B)$ is stabilizable and $(A,Q^{1/2})$ detectable,
there is a **unique** $P=P^\top\succ0$ solving the **Continuous Algebraic Riccati
Equation (CARE)**

$$
\boxed{\,A^\top P + P A - P B R^{-1} B^\top P + Q = 0\,}
\tag{12}
$$

and the optimal control is the static state feedback

$$
w^\star=-K z,\qquad K=R^{-1}B^\top P,
\tag{13}
$$

with optimal cost $J^\star=z_0^\top P z_0$ and Hurwitz closed loop $A-BK$.

### 3.3 Closed form for the task double integrator

For (10) with $Q=\mathrm{diag}(q_p I_6,\,q_v I_6)$ and $R=\rho I_6$, the CARE
decouples per axis into a scalar double integrator, solvable in closed form:

$$
K=\big[\,K_p^{\text{lqr}}\ \ K_d^{\text{lqr}}\,\big],\qquad
K_p^{\text{lqr}}=\sqrt{q_p/\rho}\;I_6,\quad
K_d^{\text{lqr}}=\sqrt{\,2\sqrt{q_p/\rho}+q_v/\rho\,}\;I_6 .
\tag{14}
$$

So $w=-K_p^{\text{lqr}}e-K_d^{\text{lqr}}\dot e$: **LQR on the feedback-linearized
arm is exactly an optimally-tuned task-space PD** — the same structure as the
impedance law (8), but with gains derived from $(Q,R)$ rather than guessed. This is
the cleanest way to *tune* the existing controller, and the intended first LQR
plugin. (Derivation of (14): solve (12) with $P=\begin{bmatrix}p_{11}I&p_{12}I\\p_{12}I&p_{22}I\end{bmatrix}$;
$p_{12}=\sqrt{\rho q_p}$, $p_{22}=\sqrt{\rho(2\sqrt{\rho q_p}+q_v)}$, then
$K=R^{-1}B^\top P=[p_{12}/\rho,\ p_{22}/\rho]$.)

### 3.4 Applying LQR to the base (and where it breaks)

For the base, linearize the 4 actuated DoF about the hover setpoint. With
$g=0$ and drag linearized to $D_0\succeq0$, the surge/heave/roll/pitch error
$z_b=[\,e_b;\dot e_b\,]\in\mathbb R^{8}$ obeys
$\dot z_b=\begin{bmatrix}0&I\\0&-M_0^{-1}D_0\end{bmatrix}z_b+\begin{bmatrix}0\\M_0^{-1}A\end{bmatrix}u$.
LQR on this pair gives the optimal thruster station-keeping law (a drop-in
replacement for the hand-tuned PD in `BaseThrusterController`).

> **Underactuation caveat.** If one instead includes **sway/yaw** in $z_b$, the pair
> $(A,B)$ is **not stabilizable**: those columns of $B$ are zero (no thruster
> produces $F_y$ or $M_z$), so Theorem 3.1's hypothesis fails and no LQR (indeed no
> smooth feedback) stabilizes them. The rig is honest about this: those DoF are
> excluded from the controlled state. Restoring them is a *hardware* change (add a
> sway/yaw thruster → new column of $A$), after which the pair becomes stabilizable.

### 3.5 Discrete time (what actually runs at 250 Hz)

The controller is sampled at $T=4\,\text{ms}$. Discretize (10):
$A_d=\begin{bmatrix}I&TI\\0&I\end{bmatrix}$, $B_d=\begin{bmatrix}\tfrac12T^2 I\\ TI\end{bmatrix}$.
LQR then solves the **Discrete ARE (DARE)**

$$
P=A_d^\top P A_d - A_d^\top P B_d\big(R+B_d^\top P B_d\big)^{-1}B_d^\top P A_d + Q,
\qquad K=\big(R+B_d^\top P B_d\big)^{-1}B_d^\top P A_d,
\tag{15}
$$

with control $w_k=-Kz_k$ [Bertsekas 2017]. Solve (12)/(15) offline once (e.g.
`care`/`dare`, or a Hamiltonian/Schur solver) and ship the constant $K$.

---

## 4. Model Predictive Control (MPC)

MPC keeps the quadratic cost but (i) works on a **finite horizon**, (ii) enforces
**constraints** explicitly, and (iii) is **re-solved every step** (receding
horizon). This is what lets us respect torque limits ($|\tau_i|\le\bar\tau_i$),
joint limits, and thrust saturation ($|u_i|\le\bar u$) — precisely the effects that
made the hand-tuned arm "slam into its limits" earlier.

### 4.1 Finite-horizon optimal control problem

With the discrete model $z_{k+1}=A_d z_k+B_d w_k$ (§3.5), at each state
$z$ solve

$$
\begin{aligned}
V_N(z)=\min_{\mathbf w}\ & \sum_{k=0}^{N-1}\underbrace{\big(z_k^\top Q z_k+w_k^\top R w_k\big)}_{\ell(z_k,w_k)}\;+\;\underbrace{z_N^\top P_f z_N}_{V_f(z_N)}\\
\text{s.t. }\ & z_{k+1}=A_d z_k+B_d w_k,\quad z_0=z,\\
& z_k\in\mathcal X,\ w_k\in\mathcal U\ (k=0..N-1),\quad z_N\in\mathcal X_f ,
\end{aligned}
\tag{16}
$$

with $Q\succeq0$, $R\succ0$, $\mathcal X,\mathcal X_f$ closed, $\mathcal U$ compact,
all containing the origin. Because the model is linear and the cost quadratic and
the constraints polyhedral, (16) is a **convex quadratic program** [Borrelli,
Bemporad & Morari 2017] solvable in milliseconds (OSQP/qpOASES).

### 4.2 Receding-horizon control law

Let $\mathbf w^\star(z)=(w_0^\star,\dots,w_{N-1}^\star)$ solve (16). Apply only the
first move and re-solve at the next state:

$$
\kappa_N(z):=w_0^\star(z),\qquad z^+=A_d z+B_d\kappa_N(z).
\tag{17}
$$

### 4.3 Terminal ingredients (the price of stability)

A finite horizon does **not** by itself guarantee stability. The standard remedy
[Mayne, Rawlings, Rao & Scokaert 2000; Rawlings, Mayne & Diehl 2017] is to add a
**terminal cost** $V_f$, **terminal set** $\mathcal X_f$, and an implicit
**terminal controller** $\kappa_f$ satisfying:

- **(A1)** $\mathcal X_f\subseteq\mathcal X$ closed, $0\in\mathcal X_f$; $\kappa_f(z)\in\mathcal U\ \forall z\in\mathcal X_f$.
- **(A2)** $\mathcal X_f$ is **positively invariant** under $\kappa_f$: $A_dz+B_d\kappa_f(z)\in\mathcal X_f\ \forall z\in\mathcal X_f$.
- **(A3)** $V_f$ is a **local control-Lyapunov function**: for all $z\in\mathcal X_f$,
  $$V_f\big(A_dz+B_d\kappa_f(z)\big)-V_f(z)\ \le\ -\,\ell\big(z,\kappa_f(z)\big).\tag{18}$$

The canonical choice ties MPC back to §3: take $\kappa_f=-K_{\text{lqr}}$ and
$V_f(z)=z^\top P z$ with $P$ the **DARE** solution (15). Then (18) holds with
*equality* inside the unconstrained region, and $\mathcal X_f$ is chosen as the
largest sublevel set $\{z:z^\top Pz\le c\}$ on which $-K_{\text{lqr}}z\in\mathcal U$
and $z\in\mathcal X$. **MPC = constrained LQR that gracefully handles saturation.**

### 4.4 Stability of MPC

**Theorem 4.1 (nominal MPC stability [Mayne et al. 2000]).** Under (A1)–(A3), with
$\ell(z,w)\ge \alpha\|z\|^2$ for some $\alpha>0$ ($Q\succ0$) and $V_f\ge0$, the origin
is **asymptotically stable** for the closed loop (17), with region of attraction the
feasible set $\mathcal X_N$ (states for which (16) admits a solution). $V_N$ is a
Lyapunov function.

*Proof.* Three steps: recursive feasibility, then descent, then conclude.

**1. Recursive feasibility.** Let $z\in\mathcal X_N$ with optimal input
$\mathbf w^\star=(w_0^\star,\dots,w_{N-1}^\star)$ and predicted trajectory
$(z_0^\star,\dots,z_N^\star)$, $z_N^\star\in\mathcal X_f$. At the successor
$z^+=z_1^\star$ consider the **shifted candidate**

$$
\tilde{\mathbf w}=\big(w_1^\star,\dots,w_{N-1}^\star,\ \kappa_f(z_N^\star)\big).
$$

By (A2) the appended step keeps the terminal state in $\mathcal X_f$
($z_{N}^\star\in\mathcal X_f\Rightarrow z_{N+1}=A_dz_N^\star+B_d\kappa_f(z_N^\star)\in\mathcal X_f\subseteq\mathcal X$),
and by (A1) $\kappa_f(z_N^\star)\in\mathcal U$. Hence $\tilde{\mathbf w}$ is
**feasible** for the problem at $z^+$, so $z^+\in\mathcal X_N$: feasibility
propagates.

**2. Descent.** $V_N(z^+)\le J(z^+,\tilde{\mathbf w})$ (optimality vs. the feasible
candidate). Writing out the candidate cost and subtracting $V_N(z)$, the shared
middle terms $\sum_{k=1}^{N-1}\ell(z_k^\star,w_k^\star)$ cancel and

$$
V_N(z^+)-V_N(z)\ \le\ -\,\ell(z,w_0^\star)\;+\;\underbrace{\Big[\ell\big(z_N^\star,\kappa_f(z_N^\star)\big)+V_f(z_{N+1})-V_f(z_N^\star)\Big]}_{\le\,0\ \text{by (A3)}}.
$$

Therefore

$$
V_N(z^+)-V_N(z)\ \le\ -\,\ell(z,w_0^\star)\ \le\ -\,\alpha\|z\|^2\ <0\quad(z\neq0).
\tag{19}
$$

**3. Conclusion.** $V_N\ge0$, $V_N(0)=0$, $V_N$ is continuous and (with $\ell$
positive definite) **positive definite and radially unbounded on $\mathcal X_N$**;
(19) makes it a strict Lyapunov function. By the discrete-time Lyapunov theorem the
origin is asymptotically stable, with domain of attraction $\mathcal X_N$.
$\qquad\blacksquare$

**Corollary 4.2 (equivalence).** With no active constraints and $N\to\infty$ (or
$V_f=$ DARE cost with $\kappa_f=-K_{\text{lqr}}$), $\kappa_N\equiv-K_{\text{lqr}}$:
MPC recovers LQR. Thus LQR is the "inner" law MPC defends when saturation is
inactive; the two share the same certificate $z^\top Pz$.

### 4.5 Robust / disturbed case

Riptide's whole point is disturbances $\tau_{\mathrm{ext}}\neq0$, i.e.
$z^+=A_dz+B_dw+E d$. Nominal Theorem 4.1 then only gives **input-to-state
stability (ISS)**: $V_N$ decreases outside a residual ball whose radius scales with
$\sup\|d\|$ [Rawlings, Mayne & Diehl 2017, §3.5]. Tightening this is **tube MPC**
(keep a nominal trajectory + an ancillary feedback $K$ that keeps the true state in
a robust invariant "tube" $z_k\in\bar z_k\oplus\mathcal S$), which is the principled
next step for the rig and a clean thesis-sized subproject.

---

## 5. The coupled two-controller system (honest caveat)

The rig runs **two** controllers at once on **one** coupled plant: the arm impedance
law and the base thruster law. Proving stability of each subsystem in isolation
(§2–§4) does **not** immediately prove stability of the interconnection — arm motion
reacts on the base and vice versa. Two rigorous routes:

1. **Passivity interconnection.** Each subsystem, with gravity/Coriolis
   compensation, is **output-strictly passive** from applied wrench to velocity
   (the storage functions are the $V$'s above; drag $D\succeq0$ only adds
   dissipation). The **passivity theorem** [van der Schaft 2017; Ortega et al.]
   states that the negative feedback interconnection of passive systems is passive,
   hence $L_2$-stable — provided the coupling is itself passive (a power-preserving
   $J^\top/J$ pairing, which the operational-space structure gives). This is the
   cleanest correctness argument for the combined rig.
2. **Singular perturbation / time-scale separation.** The hull is heavy
   ($m_b=80\,\text{kg}$) and thruster-limited → **slow**; the arm is light and
   high-bandwidth → **fast**. Treat the base as a quasi-static "boundary layer" for
   the arm and the arm as instantaneously settled for the base [Kokotović, Khalil &
   O'Reilly 1986; Khalil, *Nonlinear Systems*]. Tikhonov's theorem then certifies
   the cascade if each reduced/boundary-layer subsystem is exponentially stable —
   which §2–§4 provide. This also *justifies* the current decentralized design.

Either way, the take-home is that decentralized arm+base control is **provably**
sound under explicit coupling assumptions — and quantifying when those assumptions
fail (aggressive arm motions that violate time-scale separation) is a concrete
experiment the rig can run.

---

## 6. Where each object lives in the code

| Symbol / result | Code |
|---|---|
| $M,\;C\nu+g$ (arm) | `riptide_dynamics/PinocchioModel` (`crba`, `nonLinearEffects`) |
| $J$ (EE Jacobian, world) | `PinocchioModel::jacobian` (`getFrameJacobian`, `LOCAL_WORLD_ALIGNED`) |
| $M_A,\;D(\nu)\nu$ (fluid) | MuJoCo `fluidshape="ellipsoid"` on the hull (`generate_scene.py`) |
| $\tau_{\mathrm{ext}}$ | `/riptide/disturbance` → `xfrc_applied` (`mujoco_system.cpp`) |
| Impedance law (8) | `riptide_control/TaskSpaceImpedance` |
| Allocation $A$, eq. (5) | `BaseThrusterController::on_configure` (pinv), `update` (saturate) |
| LQR gain $K$ (planned) | new `IControlLaw` plugin: solve CARE/DARE offline, apply (13) |
| MPC QP (16) (planned) | new `IControlLaw` plugin: build/solve the QP each cycle (OSQP) |

Adding LQR or MPC is a **new `IControlLaw` plugin** — the seam, dynamics
(`PinocchioModel`), and the state/target structs are already in place.

---

## 7. References

1. T. I. Fossen. *Handbook of Marine Craft Hydrodynamics and Motion Control.* Wiley, 2011.
2. G. Antonelli. *Underwater Robots.* 3rd ed., Springer, 2014.
3. I. Schjølberg, T. I. Fossen. "Modelling and control of underwater vehicle–manipulator systems." *Proc. MCMC*, 1994.
4. R. Featherstone. *Rigid Body Dynamics Algorithms.* Springer, 2008.
5. R. M. Murray, Z. Li, S. S. Sastry. *A Mathematical Introduction to Robotic Manipulation.* CRC, 1994.
6. O. Khatib. "A unified approach for motion and force control of robot manipulators: The operational space formulation." *IEEE J. Robotics and Automation*, 3(1), 1987.
7. N. Hogan. "Impedance control: An approach to manipulation, Parts I–III." *ASME J. Dyn. Sys., Meas., Control*, 107, 1985.
8. R. Ortega, M. W. Spong. "Adaptive motion control of rigid robots: A tutorial." *Automatica*, 25(6), 1989.
9. R. E. Kalman. "Contributions to the theory of optimal control." *Bol. Soc. Mat. Mexicana*, 5, 1960.
10. B. D. O. Anderson, J. B. Moore. *Optimal Control: Linear Quadratic Methods.* Prentice-Hall, 1990.
11. F. L. Lewis, D. Vrabie, V. L. Syrmos. *Optimal Control.* 3rd ed., Wiley, 2012.
12. D. P. Bertsekas. *Dynamic Programming and Optimal Control.* Athena Scientific, 2017.
13. D. Q. Mayne, J. B. Rawlings, C. V. Rao, P. O. M. Scokaert. "Constrained model predictive control: Stability and optimality." *Automatica*, 36(6), 2000.
14. J. B. Rawlings, D. Q. Mayne, M. Diehl. *Model Predictive Control: Theory, Computation, and Design.* 2nd ed., Nob Hill, 2017.
15. F. Borrelli, A. Bemporad, M. Morari. *Predictive Control for Linear and Hybrid Systems.* Cambridge Univ. Press, 2017.
16. T. A. Johansen, T. I. Fossen. "Control allocation — A survey." *Automatica*, 49(5), 2013 (see also Fossen & Johansen 2006).
17. P. V. Kokotović, H. K. Khalil, J. O'Reilly. *Singular Perturbation Methods in Control.* Academic Press, 1986.
18. H. K. Khalil. *Nonlinear Systems.* 3rd ed., Prentice-Hall, 2002. (Lyapunov, LaSalle, Barbalat, ISS.)
19. A. van der Schaft. *L2-Gain and Passivity Techniques in Nonlinear Control.* 3rd ed., Springer, 2017.
20. J.-J. E. Slotine, W. Li. *Applied Nonlinear Control.* Prentice-Hall, 1991.
