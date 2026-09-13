# Robotic Manipulator (MATLAB)

Robotics coursework for a **6-DOF Puma-style arm** and a **2-DOF prismatic–revolute workshop arm**: Peter Corke **Robotics Toolbox** labs (`SerialLink`, `mdl_puma560`, `ikine6s`), a **quintic polynomial** trajectory solve (`ass1.m`), and an **App Designer** GUI for pose, forward/inverse kinematics, DH table, workspace animation, and joint-limit checks.

**Author:** Mohammad Rohaan · **Roll:** 22I-2327 · **GitHub:** [rohaan2802](https://github.com/rohaan2802)  
**Repo:** [Robotic-Manipulator](https://github.com/rohaan2802/Robotic-Manipulator) · **Branch:** `main` · **Language:** MATLAB  
**Verified on:** MATLAB **R2024a** + **Robotics System Toolbox** + Corke **Robotics Toolbox for MATLAB 10.4**

---

## Table of contents

1. [Features](#features)
2. [Screenshots / project demo](#screenshots--project-demo)
3. [Problem and context](#problem-and-context)
4. [Repository layout](#repository-layout)
5. [Architecture overview](#architecture-overview)
6. [Workshops — `Robot_.m`](#workshops--robot_m)
7. [Workshop 2 — `ws#02.m` and `qz` / `qr`](#workshop-2--ws02m-and-qz--qr)
8. [Assignment #02 — quintic (`ass1.m`)](#assignment-02--quintic-ass1m)
9. [Assignment #01 — App Designer](#assignment-01--app-designer)
10. [Test cases](#test-cases)
11. [Supporting PDFs and media](#supporting-pdfs-and-media)
12. [How to run](#how-to-run)
13. [Regenerating screenshots](#regenerating-screenshots)
14. [Limitations](#limitations)
15. [Author](#author)

---

## Features

| Area | What you get |
|------|----------------|
| **Assignment #01 GUI** | Maximized App Designer UI (`assignment_app.m`) with PCB backdrop (`R2.jpg`), five action buttons, temporary TextArea results, Puma `show` figures |
| **Pose helper** | Planar 2-link FK → 4×4 homogeneous matrix, then Puma joint-limit check + 3D visualization |
| **Forward kinematics** | `loadrobot('puma560')` + `getTransform` → XYZ (+ singularity probe via Corke `jacob0`) |
| **Inverse kinematics** | Workspace gate (radial / angle / \(z\)) then `inverseKinematics` on body `link6` |
| **DH + animation** | On-screen DH table, `trplot` frames, timed semicircle workspace reach |
| **Assignment #02** | Console quintic solve \(A\mathbf{a}=\mathbf{b}\) for rest-to-rest motion \(t\in[3,8]\) |
| **Workshop `Robot_.m`** | Corke `SerialLink` **NAO_ROB** (prismatic + revolute), `plot` / `teach` / `fkine` |
| **Workshop `ws#02.m`** | Puma `qz`, desired frame \(T\), `trplot`, closed-form `ikine6s` |
| **Demo gallery** | **25** live MATLAB screenshots under [`docs/screenshots/`](docs/screenshots/) |

---

## Screenshots / project demo

**25 live MATLAB captures** in [`docs/screenshots/`](docs/screenshots/) — Assignment #01, Assignment #02, and workshops, including real test-case runs plus **GUI robot poses / animation / joint-limit** shots (MATLAB R2024a + Corke Toolbox + Robotics System Toolbox).

### 1. App background (`R2.jpg`)

Industrial PCB / pick-and-place theme used as the Assignment #01 app backdrop.

![01 App background](docs/screenshots/01-app-ui-background.jpg)

### 2. Assignment #02 — boundary inputs

Test case: \(t_0=3\), \(t_f=8\), rest≈0 → rest at \(\pi/2\) (`ass1.m`).

![02 Ass2 inputs](docs/screenshots/02-ass2-boundary-inputs.png)

### 3. Assignment #02 — matrix equation \(A\mathbf{a}=\mathbf{b}\)

Vandermonde-style rows for position / velocity / acceleration at \(t_0\) and \(t_f\).

![03 Ass2 matrix](docs/screenshots/03-ass2-matrix-equation.png)

### 4. Assignment #02 — Command Window run

Live coefficients \(a_5\ldots a_0\) and the script’s `WHEN T=3/8` prints.

![04 Ass2 console](docs/screenshots/04-ass2-console-run.png)

### 5. Assignment #02 — quintic trajectory plots

Position, velocity, and acceleration over \([3, 8]\) (smooth rest-to-rest motion).

![05 Quintic trajectory](docs/screenshots/05-ass2-quintic-trajectory.png)

### 6. Assignment #01 — App Designer main UI

`assignment_app` home: POSE, FK, IK, DH & Animation, Exit.

![06 App main UI](docs/screenshots/06-ass1-app-main-ui.png)

### 7. Assignment #01 — POSE test case

Angles **30°, 45°, 10°, 0°** → planar helper 4×4 pose / XYZ.

![07 POSE test](docs/screenshots/07-ass1-pose-test.png)

### 8. Assignment #01 — IK workspace reject

Target \((0.20, 0.10, 0.30)\) fails radial rule \(0.5 < r < 1\) (expected error).

![08 IK workspace error](docs/screenshots/08-ass1-ik-workspace-error.png)

### 9. Assignment #01 — IK workspace accept

Target \((0.7, 0.2, 0.3)\) passes radial / angle / \(z\ge 0\) checks.

![09 IK valid target](docs/screenshots/09-ass1-ik-valid-target.png)

### 10. Assignment #01 — FK XYZ output

Joint test **[20 30 −15 10 0 0]°** → end-effector XYZ via `getTransform`.

![10 FK XYZ](docs/screenshots/10-ass1-fk-xyz-output.png)

### 11. Assignment #01 — FK robot figure

Puma 560 `show` for the forward-kinematics test pose.

![11 FK robot](docs/screenshots/11-ass1-fk-robot-figure.png)

### 12. Assignment #01 — IK joint solution

`inverseKinematics` solution for target \((0.7, 0.2, 0.3)\).

![12 IK joints](docs/screenshots/12-ass1-ik-joint-output.png)

### 13. Assignment #01 — IK robot figure

Puma 560 pose after the IK solve.

![13 IK robot](docs/screenshots/13-ass1-ik-robot-figure.png)

### 14. Assignment #01 — DH table

Six-row DH parameters shown by the Animation button.

![14 DH table](docs/screenshots/14-ass1-dh-table.png)

### 15. Assignment #01 — workspace reach arc

Animation concept: semicircle reach in the plane (\(r=1\)).

![15 Workspace reach](docs/screenshots/15-ass1-workspace-reach.png)

### 16. Workshop — `Robot_.m` NAO_ROB plot

Corke `SerialLink` arm at `plot([4, 0.3])` (prismatic + revolute).

![16 NAO plot](docs/screenshots/16-ws-nao-seriallink.png)

### 17. Workshop — `Robot_.m` fkine

`arm.fkine([4, 0.3])` → homogeneous \(T\) and XYZ.

![17 NAO fkine](docs/screenshots/17-ws-nao-fkine.png)

### 18. Workshop — `ws#02` Puma `plot(qz)`

Toolbox Puma 560 at the zero / home configuration.

![18 Puma qz](docs/screenshots/18-ws-puma-qz.png)

### 19. Workshop — `ws#02` ikine6s test

\(T=\mathrm{transl}(0.6,0.1,0)\,*\,\mathrm{rpy2tr}(0,180,0,\texttt{'deg'})\) → joint vector `q`.

![19 ikine6s](docs/screenshots/19-ws-puma-ikine6s.png)

### 20. Workshop — IK pose + `trplot(T)`

Puma plotted at the IK solution with the desired frame drawn.

![20 IK + trplot](docs/screenshots/20-ws-puma-ik-trplot.png)

### 21. GUI — joint rotation limits

Puma 560 min/max joint limits (Ass1 POSE / FK checks) with a mid-range arm pose inside those limits.

![21 Joint rotation limits](docs/screenshots/21-gui-joint-rotation-limits.png)

### 22. GUI — animation keyframes

Four workspace-reach frames (start → end) showing arm positions during Ass1-style animation.

![22 Animation keyframes](docs/screenshots/22-gui-animation-keyframes.png)

### 23. GUI — animation path mid-frame

End-effector arc (magenta) with the robot frozen mid-animation; green = start, red = end.

![23 Animation path](docs/screenshots/23-gui-animation-path-midframe.png)

### 24. GUI — robot arm positions

Home, ready, reach, and folded poses used across demo / animation.

![24 Arm positions](docs/screenshots/24-gui-arm-positions-grid.png)

### 25. GUI — NAO animation + rotation limits

Workshop `NAO_ROB` mid-motion with prismatic stroke limits and revolute \(q_2\in[-\pi,\pi]\) annotated.

![25 NAO animation limits](docs/screenshots/25-gui-nao-animation-limits.png)

---

## Problem and context

### Course goals

1. Build and plot serial arms with Corke **DH / `Link` / `SerialLink`**.
2. Understand **home (`qz`)** vs **ready (`qr`)** configurations and closed-form Puma IK (`ikine6s`).
3. Solve a **quintic** joint trajectory from boundary conditions (position, velocity, acceleration at \(t_0\) and \(t_f\)).
4. Ship an interactive **App Designer** tool that ties pose, FK, IK, DH, animation, and joint limits together.

### What “success” looks like

| Module | Success criteria |
|--------|------------------|
| Ass2 | \(A\) invertible; reconstructed \(q(t_0)\approx 0\), \(q(t_f)\approx\pi/2\); \(v,a\approx 0\) at both ends |
| Ass1 POSE | Numeric angles → 4×4 pose TextArea + Puma figure if joints in limits |
| Ass1 FK | Joint vector → XYZ TextArea + `show` |
| Ass1 IK | Out-of-workspace → `errordlg`; in-workspace → joint degrees + `show` |
| Ass1 Animation | DH table visible; frames animate along semicircle |
| `Robot_.m` | NAO_ROB plots at `[4,0.3]`; teach pendant opens |
| `ws#02.m` | After one-line `T` fix: `qz` plot, `trplot(T)`, finite `ikine6s` solution |

---

## Repository layout

```text
Robotic-Manipulator/
├── README.md
├── assignment_app_1.txt          # text mirror of Assignment #01 app
├── Assignment #01/
│   ├── assignment_app.m          # App Designer source (portable R2.jpg path)
│   ├── R2.jpg                    # GUI background
│   ├── Report.pdf
│   └── Humanoid Robot Pitch Deck by Slidesgo.pptx
├── Assignment #02/
│   ├── ass1.m                    # quintic A*a = b
│   └── SEC.pdf                   # assignment brief
├── WORK_SHOPS/
│   ├── Robot_.m                  # NAO_ROB SerialLink lab
│   ├── ws#02.m                   # Puma qz + ikine6s (fix T line before run)
│   ├── qz and qr.txt             # student notes
│   └── *.pdf                     # workshop / toolbox / ESP8266 handouts
└── docs/
    ├── capture_screenshots.m     # regenerate shots 01–20
    ├── capture_gui5.m            # regenerate shots 21–25
    └── screenshots/              # README gallery PNGs/JPGs + quintic_results.txt
```

---

## Architecture overview

```text
┌─────────────────────────────────────────────────────────────┐
│ Assignment #01  assignment_app.m  (AppBase UI)              │
│  POSE ──► planar FK helper ──► TextArea ──► Puma show       │
│  FK   ──► loadrobot + getTransform(link4) ──► TextArea/show │
│  IK   ──► workspace gates ──► inverseKinematics(link6)      │
│  Anim ──► DH uitable ──► trplot ──► timer semicircle        │
└─────────────────────────────────────────────────────────────┘
┌──────────────────────┐  ┌───────────────────────────────────┐
│ Assignment #02       │  │ Workshops                         │
│ ass1.m  A \ b → a_i  │  │ Robot_.m : Link/SerialLink/teach  │
│ quintic q(t) coeffs  │  │ ws#02.m  : mdl_puma560/ikine6s    │
└──────────────────────┘  └───────────────────────────────────┘
```

**Tooling split**

| Library | Used for |
|---------|----------|
| Corke Robotics Toolbox | `Link`, `SerialLink`, `mdl_puma560`, `fkine`, `ikine6s`, `trplot`, `transl`, `rpy2tr`, `jacob0` |
| Robotics System Toolbox | `loadrobot`, `show`, `getTransform`, `inverseKinematics` |
| App Designer | `uifigure`, buttons, TextArea, `UIAxes2` backdrop |

---

## Workshops — `Robot_.m`

File: [`WORK_SHOPS/Robot_.m`](WORK_SHOPS/Robot_.m).

An early DH matrix is defined then left unused (commented experiments with `SerialLink(dh)` / two-revolute NAO):

```matlab
dh = [
0 0 1 0
0 0 1 0
]
```

**Active code** — 2-DOF **prismatic + revolute** arm named `NAO_ROB`:

```matlab
L(1) = Link([0 2 0 pi/2 1], 'standard');  % last flag 1 = prismatic
L(2) = Link([0 0 3 0 0], 'standard');     % revolute
L(1).qlim = [0, 5];
arm = SerialLink(L, 'name', 'NAO_ROB');
arm.plot([4, 0.3])
arm.teach
```

| Link | θ | d | a | α | Joint | Limit |
|------|---|---|---|---|--------|-------|
| 1 | 0 | 2 | 0 | π/2 | Prismatic | `qlim` **[0, 5]** |
| 2 | 0 | 0 | 3 | 0 | Revolute | default (demo annotates \([-\pi,\pi]\)) |

Demo configuration: prismatic extension **4**, revolute **0.3 rad**. `teach` opens the interactive teach pendant (block until closed).

**Verified FK** for `q = [4, 0.3]`:

\[
{}^{0}T_{2} \Rightarrow \text{XYZ} \approx [2.866,\ 0.000,\ 4.887]
\]

See screenshots **16**, **17**, **25**.

---

## Workshop 2 — `ws#02.m` and `qz` / `qr`

[`WORK_SHOPS/ws#02.m`](WORK_SHOPS/ws#02.m) (as stored):

```matlab
mdl_puma560
p560.plot(qz)
T = transl(0.6, 0.1, 0)
* rpy2tr(0, 180, 0, 'deg');
hold on
trplot(T)
q = p560.ikine6s(T)
```

**Fix before running** — join the transform into one statement:

```matlab
T = transl(0.6, 0.1, 0) * rpy2tr(0, 180, 0, 'deg');
```

Intended flow: load Puma 560 → plot **zero** pose `qz` → build frame at \((0.6, 0.1, 0)\) with RPY \((0^\circ,180^\circ,0^\circ)\) → `trplot` → closed-form IK `ikine6s`.

**Verified `ikine6s` solution** (Toolbox Puma):

```text
q ≈ [3.0575  2.3210  0.1162  -0.0000  0.7044  3.0575]
```

### `qz` / `qr` notes ([`qz and qr.txt`](WORK_SHOPS/qz%20and%20qr.txt))

| Symbol | Meaning |
|--------|---------|
| `qz` | Zero / home joints (all zeros). 2-link example `[0,0]`; Puma is a **6-vector**. |
| `qr` | Ready / task configuration (notes example `[0.2, 0.3]` rad for 2-link). |

Typical use: `fkine(qz)`, `fkine(qr)`, and motion planning from `qz` → `qr`. Screenshots **18–20**.

---

## Assignment #02 — quintic (`ass1.m`)

File: [`Assignment #02/ass1.m`](Assignment%20%2302/ass1.m). Brief: [`SEC.pdf`](Assignment%20%2302/SEC.pdf).

Solves \(\mathbf{a} = A^{-1}\mathbf{b}\) for

\[
q(t) = a_5 t^5 + a_4 t^4 + a_3 t^3 + a_2 t^2 + a_1 t + a_0
\]

Rows of \(A\) encode \(q\), \(\dot q\), \(\ddot q\) at \(t_0\) and \(t_f\).

| Symbol | Value in file | Role |
|--------|----------------|------|
| `t0` | 3 | start time |
| `tf` | 8 | end time |
| `q1` | 0 | position at \(t_0\) |
| `s1` | \(\pi/2\) | position at \(t_f\) (stored in `b`) |
| `q2` | \(2\times10^{-9}\) | velocity at \(t_0\) |
| `s2` | \(5\times10^{-9}\) | velocity at \(t_f\) |
| accel | 0, 0 | \(\ddot q(t_0)=\ddot q(t_f)=0\) |

```matlab
b = [q1; q2; 0; s1; s2; 0];
```

**Live solve** (also in [`docs/screenshots/quintic_results.txt`](docs/screenshots/quintic_results.txt)):

| Coeff | Value |
|-------|-------|
| \(a_5\) | \(0.00301592891385\) |
| \(a_4\) | \(-0.0829380451428\) |
| \(a_3\) | \(0.849486644331\) |
| \(a_2\) | \(-3.981026168\) |
| \(a_1\) | \(8.68587527841\) |
| \(a_0\) | \(-7.17941878962\) |

Boundary reconstruction check: \(q(t_0)\approx 0\), \(q(t_f)\approx 1.5708\), \(v(t_0)\approx 2\times10^{-9}\), \(v(t_f)\approx 5\times10^{-9}\), accelerations \(\approx 0\).

> Note: the script’s `WHEN T=3` / `WHEN T=8` lines print **coefficient triples**, not evaluated \(q,v,a\) at those times. Screenshots **02–05**.

---

## Assignment #01 — App Designer

**Source:** [`Assignment #01/assignment_app.m`](Assignment%20%2301/assignment_app.m) (mirror: [`assignment_app_1.txt`](assignment_app_1.txt)).  
Class `assignment_app < matlab.apps.AppBase`. Window starts **maximized**; banner **Robotic Arm Simulation**.

### UI

Five teal buttons: (1) Transformation Matrix (POSE), (2) Forward Kinematics, (3) Inverse Kinematics, (4) DH Table & Animation, (5) Exit. Results appear briefly in a large `TextArea`. Backdrop is `UIAxes2` + `R2.jpg`.

### Portable background load

```matlab
appDir = fileparts(mfilename('fullpath'));
imgFile = fullfile(appDir, 'R2.jpg');
app.BackgroundImage = imread(imgFile);
```

Keep `R2.jpg` next to `assignment_app.m` after clone/move.

### Helpers

| Method | Behaviour |
|--------|-----------|
| `forwardKinematics` | Planar 2-link \(L_1=L_2=1\); \(z=\theta_3\); returns 4×4 pose |
| `forwardKinematics_1` | 3-link planar helper — **unused** by buttons |
| `isSingular` | Corke `mdl_puma560` + `jacob0`; singular if \(\lvert\det J\rvert < 10^{-6}\) |

### Button behaviour (detail)

| Button | Inputs | Core logic | Output |
|--------|--------|------------|--------|
| **POSE** | 4 angles (deg) | planar FK → TextArea 5 s → `loadrobot` limit check → `show` (**SUMO_ROBO**) | 4×4 + robot figure |
| **FK** | 4 angles (deg) | pad to 6 DOF → `getTransform(...,'link4')` → TextArea 4 s → `show` (**SOMO**) | XYZ + robot |
| **IK** | \(X,Y,Z\) | gates then `inverseKinematics` on **`link6`**, weights `[1 1 1 0 0 0]` | 6 thetas (deg) + robot |
| **Animation** | — | DH `uitable` 10 s → `trplot` → timer semicircle \(r=1\), period 0.025 s | DH + animated frames |
| **Exit** | — | `delete(app.UIFigure)` | closes app |

### IK workspace rules (as coded)

- Radial: \(0.5 < r=\sqrt{x^2+y^2} < 1\)
- Angle: \(\operatorname{atan2d}(y,x) \in [-90, 90]\)
- Height: \(z \ge 0\)

### Display-only DH table (Animation button)

| Joint | θ | d | a | α |
|-------|---|---|---|---|
| 1 | 0 | 0.2 | 0.5 | −90 |
| 2 | 0 | 0 | 0.5 | 0 |
| 3 | 0 | 0 | 0.5 | 0 |
| 4 | 0 | 0.1 | 0 | −90 |
| 5 | 0 | 0 | 0 | 90 |
| 6 | 0 | 0 | 0 | 0 |

### Puma joint limits used in POSE / FK checks

Typical MathWorks `puma560` revolute bounds (see shot **21**):

| Joint | Min | Max |
|-------|-----|-----|
| q1 | −180° | +180° |
| q2…q6 | −90° | +90° |

---

## Test cases

Run these in MATLAB (Command Window or App UI). **Expected** values were verified on R2024a with Corke 10.4 + Robotics System Toolbox. Screenshot IDs point into [`docs/screenshots/`](docs/screenshots/).

### Assignment #02 — `ass1.m`

| ID | Name | Input / action | Expected | Evidence |
|----|------|----------------|----------|----------|
| **A2-T1** | Boundary pack | Defaults in file | \(t_0=3\), \(t_f=8\), \(q(t_0)=0\), \(q(t_f)=\pi/2\), near-zero velocities, zero accels | Shot **02** |
| **A2-T2** | Matrix solve | `run('ass1.m')` | Finite `inv(A)`; 6 coeffs printed; \(\det(A)\approx -7.81\times10^6\) | Shots **03**, **04** |
| **A2-T3** | Coeff dump | Compare to `quintic_results.txt` | \(a_5\ldots a_0\) match table above (±1e-9) | Shot **04** + txt |
| **A2-T4** | Rest-to-rest check | Evaluate poly with `val` on `linspace(3,8)` | \(q(3)\approx 0\), \(q(8)\approx\pi/2\); \(v,a\approx 0\) at ends; smooth S-curve | Shot **05** |
| **A2-T5** | WHEN labels | Read Command Window | Prints labelled triples (coeffs), does **not** assert motion at t=3/8 | Shot **04** |

```matlab
cd('Assignment #02');
run('ass1.m');
```

### Assignment #01 — App / kinematics

| ID | Name | Input / action | Expected | Evidence |
|----|------|----------------|----------|----------|
| **A1-T1** | App launch | `assignment_app` from `Assignment #01` | Maximized UI, title **Robotic Arm Simulation**, `R2.jpg` backdrop, 5 buttons | Shot **06** (+ **01**) |
| **A1-T2** | POSE nominal | Angles **30, 45, 10, 0** deg | Pose TextArea; XYZ ≈ **[1.125, 1.466, 0.175]**; optional Puma figure if limits OK | Shot **07** |
| **A1-T3** | POSE bad input | Non-numeric / empty cancel | `errordlg` or silent return; no crash | Manual |
| **A1-T4** | FK nominal | Joints **[20 30 −15 10]** deg (+ pads 0,0) | XYZ TextArea + Puma `show` | Shots **10**, **11** |
| **A1-T5** | FK joint limit | Angle outside body PositionLimits | `errordlg` with joint index + range | Manual / shot **21** limits |
| **A1-T6** | IK reject (radial) | \((0.20, 0.10, 0.30)\) → \(r\approx 0.224 < 0.5\) | Workspace `errordlg` (radial) | Shot **08** |
| **A1-T7** | IK reject (z) | e.g. \((0.7, 0.2, -0.1)\) | Workspace `errordlg` (z) | Manual |
| **A1-T8** | IK reject (angle) | Point with \(\lvert\operatorname{atan2d}(y,x)\rvert>90\) | Workspace `errordlg` (angular) | Manual |
| **A1-T9** | IK accept | \((0.7, 0.2, 0.3)\) → \(r\approx 0.728\) | Passes gates; 6 joint degrees; robot figure | Shots **09**, **12**, **13** |
| **A1-T10** | DH table | Button **4** | 6-row DH uitable (~10 s) | Shot **14** |
| **A1-T11** | Workspace animation | Continue after DH | Semicircle reach \(r=1\); animated `trplot` frames | Shots **15**, **22**, **23** |
| **A1-T12** | Exit | Button **5** | Figure closes cleanly | Manual |
| **A1-T13** | Missing `R2.jpg` | Rename/move image then launch | `errordlg` “R2.jpg not found…” | Manual |

```matlab
cd('Assignment #01');
assignment_app;
% or: appdesigner('assignment_app.m');
```

### Workshops — `Robot_.m`

| ID | Name | Input / action | Expected | Evidence |
|----|------|----------------|----------|----------|
| **W1-T1** | Build arm | Run `Robot_.m` (or equivalent without `teach`) | `NAO_ROB` SerialLink created; `qlim` on link 1 = `[0,5]` | Shot **16** |
| **W1-T2** | Plot config | `arm.plot([4, 0.3])` | Prismatic+revolute figure, checkered floor | Shot **16** |
| **W1-T3** | Forward kinematics | `arm.fkine([4, 0.3])` | XYZ ≈ **[2.866, 0.000, 4.887]** | Shot **17** |
| **W1-T4** | Stroke limit | `q1=4` inside `[0,5]`; try `q1=6` conceptually | Valid demo uses interior of prismatic stroke | Shot **25** |
| **W1-T5** | Teach pendant | Uncomment/`arm.teach` | Interactive sliders; blocks until closed | Manual |
| **W1-T6** | Mid-motion annotation | Animate `q` from `[1,-0.4]` → `[4,0.8]` | Mid frame shows limits callouts | Shot **25** |

```matlab
cd('WORK_SHOPS');
% Prefer: build arm then plot/fkine; use teach only when interactive
L(1) = Link([0 2 0 pi/2 1],'standard');
L(2) = Link([0 0 3 0 0],'standard');
L(1).qlim = [0 5];
arm = SerialLink(L,'name','NAO_ROB');
arm.plot([4 0.3]);
T = arm.fkine([4 0.3]);
```

### Workshops — `ws#02.m` / Puma

| ID | Name | Input / action | Expected | Evidence |
|----|------|----------------|----------|----------|
| **W2-T1** | Syntax fix | One-line `T = transl(...)*rpy2tr(...)` | Script parses | — |
| **W2-T2** | Home plot | `mdl_puma560`; `p560.plot(qz)` | Puma at all-zero joints | Shot **18** |
| **W2-T3** | Target frame | \(T\) at (0.6, 0.1, 0), RPY (0,180,0) deg | `trplot(T)` overlays frame | Shot **20** |
| **W2-T4** | Closed-form IK | `q = p560.ikine6s(T)` | Finite 6-vector ≈ `[3.06 2.32 0.12 0 0.70 3.06]` | Shot **19** |
| **W2-T5** | Plot IK pose | `p560.plot(q)` + `trplot(T)` | Arm reaches near desired frame | Shot **20** |
| **W2-T6** | `qz`/`qr` notes | Read `qz and qr.txt` | Home vs ready definitions understood | Text file |

```matlab
cd('WORK_SHOPS');
mdl_puma560
p560.plot(qz)
T = transl(0.6, 0.1, 0) * rpy2tr(0, 180, 0, 'deg');
hold on; trplot(T)
q = p560.ikine6s(T)
p560.plot(q)
```

### GUI / animation gallery cases

| ID | Name | Expected | Evidence |
|----|------|----------|----------|
| **G-T1** | Limits dashboard | Min/max degrees listed; mid-range pose | Shot **21** |
| **G-T2** | Keyframe grid | 4 distinct reach poses | Shot **22** |
| **G-T3** | Path mid-frame | Magenta EE arc; green start / red end | Shot **23** |
| **G-T4** | Pose collage | Home / ready / reach / folded | Shot **24** |

### Quick pass/fail checklist

- [ ] Ass2 prints six coefficients matching `quintic_results.txt`
- [ ] Ass1 opens with background image
- [ ] Ass1 IK rejects \((0.2,0.1,0.3)\) and accepts \((0.7,0.2,0.3)\)
- [ ] NAO `fkine([4,0.3])` XYZ ≈ `[2.87, 0, 4.89]`
- [ ] Fixed `ws#02` returns a finite `ikine6s` vector

---

## Supporting PDFs and media

| Path | Role |
|------|------|
| `Assignment #01/Report.pdf` | Written report |
| `Assignment #01/Humanoid Robot Pitch Deck by Slidesgo.pptx` | Pitch deck (Slidesgo template) |
| `Assignment #01/R2.jpg` | App background (portable `fullfile` + `mfilename`) |
| `Assignment #02/SEC.pdf` | Quintic assignment brief |
| `WORK_SHOPS/Robotics Workshop 1 .pdf` | Workshop 1 |
| `WORK_SHOPS/Workshop 2+3 Pose(2D,3D).pdf` | Pose 2D/3D |
| `WORK_SHOPS/Week 6 - Workshop 4 - Complete Simulation.pdf` (+ `_2`) | Simulation write-up |
| `WORK_SHOPS/robotics tool box manual.pdf` (+ `_2`) | Toolbox manuals |
| `WORK_SHOPS/Tutorial Demo on ESP8266 - 1.pdf` … `- 3.pdf` | Hardware Wi-Fi tutorials — **not** required for the `.m` app |

---

## How to run

**Requirements**

- MATLAB **R2020b+** (verified **R2024a**)
- [Peter Corke Robotics Toolbox](https://petercorke.com/toolboxes/robotics-toolbox/) (`.mltbx` / Add-On)
- **Robotics System Toolbox** (`loadrobot`, `getTransform`, `inverseKinematics`, `show`)
- App Designer for editing `assignment_app.m`

```matlab
%% Workshops
cd('WORK_SHOPS');
% Robot_.m ends in teach (blocking) — use plot/fkine snippet from Test cases for batch demos
run('Robot_.m');

% Fix T onto one line, then:
% T = transl(0.6,0.1,0)*rpy2tr(0,180,0,'deg');
run('ws#02.m');

%% Assignment #02
cd('../Assignment #02');
run('ass1.m');

%% Assignment #01
cd('../Assignment #01');
assignment_app;
% appdesigner('assignment_app.m');
```

`R2.jpg` must remain beside `assignment_app.m`.

---

## Regenerating screenshots

From the repo root (non-interactive batch):

```matlab
cd('path/to/Robotic-Manipulator');
run('docs/capture_screenshots.m');   % shots 01–20 + quintic_results.txt
run('docs/capture_gui5.m');          % shots 21–25 (GUI poses / animation / limits)
```

Or from PowerShell:

```powershell
matlab -batch "cd('C:/path/to/Robotic-Manipulator'); run('docs/capture_screenshots.m'); run('docs/capture_gui5.m')"
```

---

## Limitations

- App mixes a **2-link planar** pose helper with a **6-DOF Puma** visualiser; FK uses body **`link4`**, IK uses **`link6`**.
- `forwardKinematics_1` is unused; singularity `sprintf` results are discarded.
- `ws#02.m` as committed has a broken line break on `T = … * rpy2tr`; join before run.
- `ass1.m` “WHEN T=…” prints coefficient groups, not motion evaluated at \(t=3\) or \(t=8\).
- ESP8266 PDFs are unrelated to the manipulator scripts.
- Do not commit a full Toolbox `rvctools` tree into this repo.
- README gallery: **25** live captures (`docs/capture_screenshots.m` + `docs/capture_gui5.m`).

---

## Author

**Mohammad Rohaan** · Roll **22I-2327** · [github.com/rohaan2802](https://github.com/rohaan2802)
