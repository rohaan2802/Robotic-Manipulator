# Robotic Manipulator (MATLAB)

Robotics coursework: **Peter Corke Robotics Toolbox** workshops (`SerialLink` / Puma 560), a **quintic polynomial** coefficient solve, and an **App Designer** app for pose, forward/inverse kinematics, DH table, and a simple animation.

**Author:** Mohammad Rohaan · **Roll:** 22I-2327 · **GitHub:** [rohaan2802](https://github.com/rohaan2802)

---

## Table of contents

1. [Screenshots / project demo](#screenshots--project-demo)
2. [Problem and context](#problem-and-context)
3. [Workshops — `Robot_.m`](#workshops--robot_m)
4. [Workshop 2 — `ws#02.m` and `qz` / `qr`](#workshop-2--ws02m-and-qz--qr)
5. [Assignment #02 — quintic (`ass1.m`)](#assignment-02--quintic-ass1m)
6. [Assignment #01 — App Designer](#assignment-01--app-designer)
7. [Supporting PDFs and media](#supporting-pdfs-and-media)
8. [How to run](#how-to-run)
9. [Limitations](#limitations)
10. [Author](#author)

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

Folder layout on GitHub:

```text
docs/screenshots/   # README demo PNGs/JPGs
Assignment #01/     # App Designer + report + pitch deck
Assignment #02/     # Quintic polynomial (SEC.pdf + ass1.m)
WORK_SHOPS/         # Toolbox labs, ESP8266 PDFs, pose PDFs
assignment_app_1.txt  # Text copy of assignment_app.m
```

Language (GitHub): MATLAB. Default branch: `main`. This working copy has `ass1.m`, `Robot_.m`, and `assignment_app_1.txt` at the folder root; nested paths above are the canonical tree.

---

## Workshops — `Robot_.m`

File: `WORK_SHOPS/Robot_.m`.

A DH matrix is defined then left unused:

```matlab
dh = [
0 0 1 0
0 0 1 0
]
```

Commented experiments: `SerialLink(dh)` / `'name','Nao'` with `plot`/`teach`/`fkine([0.2 0.3])`, and a two-revolute `NAO_ROB` (`[0 2 0 pi/2 0]`, `[0 0 2 0]`).

**Active code** — 2-DOF **prismatic + revolute** arm named `NAO_ROB`:

```matlab
L(1) = Link([0 2 0 pi/2 1], 'standard');  % last flag 1 = prismatic
L(2) = Link([0 0 3 0 0], 'standard');     % revolute
L(1).qlim = [0, 5];
arm = SerialLink(L, 'name', 'NAO_ROB');
arm.plot([4, 0.3])
arm.teach
```

| Link | θ | d | a | α | Joint |
|------|---|---|---|---|--------|
| 1 | 0 | 2 | 0 | π/2 | Prismatic, `qlim` [0, 5] |
| 2 | 0 | 0 | 3 | 0 | Revolute |

Plot configuration: prismatic extension **4**, revolute **0.3 rad**. `teach` opens the Toolbox teach pendant. Requires Corke Robotics Toolbox (`Link`, `SerialLink`).

---

## Workshop 2 — `ws#02.m` and `qz` / `qr`

`WORK_SHOPS/ws#02.m` (as fetched):

```matlab
mdl_puma560
p560.plot(qz)
T = transl(0.6, 0.1, 0)
* rpy2tr(0, 180, 0, 'deg');
hold on
trplot(T)
q = p560.ikine6s(T)
```

Intended behaviour: load the Toolbox **Puma 560** model, plot the **zero** pose `qz`, build a homogeneous transform at (0.6, 0.1, 0) with RPY (0°, 180°, 0°), draw it with `trplot`, and solve closed-form IK (`ikine6s`).

As stored, `T = transl(...)` is one statement and the next line begins with `* rpy2tr(...)`, which MATLAB will not parse as `T = transl(...) * rpy2tr(...)`. Join those lines before running.

`WORK_SHOPS/qz and qr.txt` (student notes):

| Symbol | Meaning in the note |
|--------|---------------------|
| `qz` | Zero / home joint vector, e.g. `[0, 0]` |
| `qr` | A ready or target configuration, example `[0.2, 0.3]` rad |

Uses described: `fkine(qz)` / `fkine(qr)`, and motion from `qz` to `qr`. For Puma 560, Toolbox `qz` / `qr` are 6-vectors, not 2-link.

---

## Assignment #02 — quintic (`ass1.m`)

Solves **A a = b** for coefficients of

```text
q(t) = a5 t^5 + a4 t^4 + a3 t^3 + a2 t^2 + a1 t + a0
```

Rows of `A` are q, q̇, q̈ at `t0` and at `tf` (Vandermonde-style).

| Symbol | Value in file |
|--------|----------------|
| `t0` | 3 |
| `tf` | 8 |
| `q1` | 0 (position at t0) |
| `s1` | π/2 (used in `b` as the **tf position** slot — see `b` below) |
| `q2` | 2×10⁻⁹ |
| `s2` | 5×10⁻⁹ |
| Accelerations | 0 at both ends |

```matlab
b = [q1; q2; 0; s1; s2; 0];
```

Matching `A` rows: q(t0)=`q1`, q̇(t0)=`q2`, q̈(t0)=0, q(tf)=`s1`=π/2, q̇(tf)=`s2`, q̈(tf)=0. Start is nearly rest at 0; end is nearly rest at **π/2**. Script prints `inv(A)` and `val = A_inv * b` (`a5`…`a0`). The `WHEN T=3` / `WHEN T=8` blocks **do not** evaluate motion; they print coefficient triples labelled q/v/a. Use `val` in the report (`SEC.pdf`).

---

## Assignment #01 — App Designer

**`assignment_app.m`** (also mirrored in `assignment_app_1.txt`). Class `assignment_app < matlab.apps.AppBase`. Window title **MATLAB App**, starts maximized. Banner textarea: **Robotic Arm Simulation** (Book Antiqua, cyan on dark).

### UI (`createComponents`)

Maximized figure, banner **Robotic Arm Simulation**. Five teal Cooper Black buttons: (1) Transformation Matrix (POSE), (2) Forward Kinematics, (3) Inverse Kinematics, (4) DH Table & Animation, (5) Exit. `TextArea` holds output; `UIAxes2` is the background. Declared but not created: `UIAxes`, `StartAnimationButton`, `RobotAxes`.

### Startup

`startupFcn` hides `TextArea`, then loads the backdrop with a **portable** path next to the app:

```matlab
app.BackgroundImage = imread(fullfile(fileparts(mfilename('fullpath')), 'R2.jpg'));
```

Keep `R2.jpg` in the same `Assignment #01` folder as `assignment_app.m` (works after moving/cloning the repo). `UIAxes2.Position = [-90 -110 1700 1300]`.

### Helper methods

- **`forwardKinematics(theta1, theta2, theta3, ~)`** — planar 2-link, `L1 = L2 = 1`: `x,y` from standard planar FK, `z = θ3`; returns a 4×4 pose with identity rotation.
- **`forwardKinematics_1(jointAngles)`** — 3 planar links `L1=L2=L3=1`, `z = theta4`; returns `[x; y; z]` (**unused** by the buttons).
- **`isSingular(jointAngles)`** — `mdl_puma560`; `p560.jacob0`; singular if `|det(J)| < 1e-6`.

### Button: Transformation Matrix (POSE)

`inputdlg` four angles (degrees, default 0). Converts with `deg2rad`. Calls `forwardKinematics` (2-link + z). Shows the 4×4 in `TextArea` for 5 s. Then `loadrobot('puma560', 'DataFormat','row', 'Gravity',[0 0 -9.81])`, checks the first four joints against `Bodies{i}.Joint.PositionLimits`, pads `theta = [θ1 θ2 θ3 θ4 0 0]`, `show` in a figure named **SUMO_ROBO**.

### Button: Forward Kinematics

`loadrobot('puma560', …)` again. Four-angle dialog. Pads to 6 DOF. End-effector name **`link4`**. `getTransform` + `transl` for XYZ. `isSingular` is called but its `sprintf` results are **not assigned** (warnings unused). Displays X/Y/Z for 4 s. Figure name **SOMO**.

### Button: Inverse Kinematics

Dialog: X, Y, Z. Workspace checks (as coded):

- Radial `r = hypot(x,y)` must satisfy **0.5 < r < 1** (metres).
- `atan2d(y,x)` in **[−90, 90]** degrees.
- **z ≥ 0**.

Then `transl(desired) * troty(0)`, `robotics.InverseKinematics` on the Puma tree, weights `[1 1 1 0 0 0]`, seed `zeros(1,6)`, body **`link6`**. Prints six thetas in degrees. Figure **SOMO**.

### Button: DH Table & Animation

1. Shows a `uifigure` table (10 s) with the DH rows in the next section.
2. `trplot` of `T0 = eye(4)` and `T1 = transl(1,2,3)*rpy2tr(0.6, 0.8, 1.4)`.
3. 100-point semicircle in XY, radius 1, `theta = linspace(0, pi, 100)`.
4. `timer` period **0.025 s** plots `transl(S(i,:)) * rpy2tr(0.6, 0.8, 1.4)` via nested `updateRobot`.

The DH `uitable` (display-only, not wired to `SerialLink`) uses columns Joint, θ, d, a, α:

| Joint | θ | d | a | α |
|-------|---|---|---|---|
| 1 | 0 | 0.2 | 0.5 | −90 |
| 2 | 0 | 0 | 0.5 | 0 |
| 3 | 0 | 0 | 0.5 | 0 |
| 4 | 0 | 0.1 | 0 | −90 |
| 5 | 0 | 0 | 0 | 90 |
| 6 | 0 | 0 | 0 | 0 |

### Exit

`delete(app.UIFigure)`.

---

## Supporting PDFs and media

| Path | Role |
|------|------|
| `Assignment #01/Report.pdf` | Written report |
| `Assignment #01/Humanoid Robot Pitch Deck by Slidesgo.pptx` | Pitch deck (Slidesgo template) |
| `Assignment #01/R2.jpg` | App background (loaded via `fullfile` + `mfilename`) |
| `Assignment #02/SEC.pdf` | Quintic assignment brief |
| `WORK_SHOPS/Robotics Workshop 1 .pdf` | Workshop 1 |
| `WORK_SHOPS/Workshop 2+3 Pose(2D,3D).pdf` | Pose 2D/3D |
| `WORK_SHOPS/Week 6 - Workshop 4 - Complete Simulation.pdf` (+ `_2`) | Simulation write-up |
| `WORK_SHOPS/robotics tool box manual.pdf` (+ `_2`) | Toolbox manuals |
| `WORK_SHOPS/Tutorial Demo on ESP8266 - 1.pdf` … `- 3.pdf` | Hardware Wi-Fi module tutorials — **not** required to run the `.m` app |

---

## How to run

MATLAB with **App Designer** (R2020b+ recommended) and [Peter Corke Robotics Toolbox](https://petercorke.com/toolboxes/robotics-toolbox/). Puma callbacks also need **Robotics System Toolbox** (`loadrobot`, `getTransform`, `inverseKinematics`).

```matlab
cd('WORK_SHOPS');
run('Robot_.m');           % NAO_ROB teach pendant

% After fixing the T = transl * rpy2tr line:
run('ws#02.m');            % Puma qz + ikine6s

cd('../Assignment #02');
run('ass1.m');             % prints A^{-1} and coefficients

cd('../Assignment #01');
assignment_app;              % runs the App Designer UI
% or edit in App Designer:
% appdesigner('assignment_app.m');
```

`R2.jpg` stays beside `assignment_app.m` in `Assignment #01` (portable `fullfile` path).

**Dependencies:** Corke Robotics Toolbox (`Link`, `SerialLink`, `mdl_puma560`, `ikine6s`, `trplot`, `transl`, `rpy2tr`); MATLAB Robotics System Toolbox (`loadrobot`, `getTransform`, `inverseKinematics`); App Designer for the UI.

---

## Limitations

- App mixes a **2-link planar** pose helper with a **6-DOF Puma** visualiser; FK uses `link4`, IK uses `link6`.
- `forwardKinematics_1` unused; singularity `sprintf` discarded.
- `ws#02.m` line break will error until concatenated.
- `ass1.m` “WHEN T=…” prints coefficients, not motion at t = 3 or 8.
- ESP8266 PDFs are unrelated to the manipulator scripts.
- Do not commit a full Toolbox `rvctools` tree.
- README gallery: **25** live captures (`docs/capture_screenshots.m` + `docs/capture_gui5.m`; re-run after code changes).

---

## Author

**Mohammad Rohaan** · Roll **22I-2327** · [github.com/rohaan2802](https://github.com/rohaan2802)
