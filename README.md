# Robotic Manipulator (MATLAB)

Robotics coursework: **Peter Corke Robotics Toolbox** workshops, **quintic polynomial** coefficient solve, and an **App Designer** app for FK / IK / pose / singularity / animation.

[rohaan2802](https://github.com/rohaan2802)

---

## Table of contents

1. [Workshops (`Robot_.m`)](#workshops-robot_m)
2. [Assignment #02 — quintic (`ass1.m`)](#assignment-02--quintic-ass1m)
3. [Assignment #01 — App Designer](#assignment-01--app-designer)
4. [How to run](#how-to-run)
5. [Paths](#paths)

---

## Workshops (`Robot_.m`)

Active code builds a 2-link **`SerialLink` named `NAO_ROB`**:

```matlab
L(1) = Link([0 2 0 pi/2 1], 'standard');  % prismatic (last DH flag 1)
L(2) = Link([0 0 3 0 0], 'standard');     % revolute
L(1).qlim = [0, 5];
arm = SerialLink(L, 'name', 'NAO_ROB');
arm.plot([4, 0.3])
arm.teach
```

Commented blocks show earlier 2-revolute DH, `fkine`, and a `Nao` name variant.  
`WORK_SHOPS/` also has `ws#02.m`, `qz and qr.txt`, pose 2D/3D PDFs, complete simulation PDF, **ESP8266** tutorial PDFs, toolbox manuals.

Requires **Robotics Toolbox for MATLAB** on the path.

---

## Assignment #02 — quintic (`ass1.m`)

Solves **A a = b** for a 5th-order polynomial (position, velocity, acceleration at **t0** and **tf**).

| Symbol | Value in file |
|--------|----------------|
| `t0` | 3 |
| `tf` | 8 |
| `q1` | 0 |
| `s1` | `pi/2` |
| `q2` | `2e-9` |
| `s2` | `5e-9` |
| Accel BCs | 0 at both ends (rows 3 and 6 of `b`) |

`A` is 6×6 Vandermonde-style with derivative rows. Prints `inv(A)`, coefficient vector `val` (`a5…a0`), then a slightly **mislabeled** dump (`WHEN T=3` prints `a5,a4,a3` as q/v/a — use `val` itself for the report).

This is the standard **quintic rest-to-rest** (almost: end position/vel are tiny ε, not exactly the π/2 start vel story — check the assignment PDF `SEC.pdf`).

---

## Assignment #01 — App Designer

**`assignment_app.mlapp`** — export text `assignment_app_1.txt`.

UI buttons: **Forward Kinematics**, **Inverse Kinematics**, **Transformation Matrix / POSE**, **Animation**, **Exit**, plus text areas and axes.

Helpers:

- `forwardKinematics(theta1,theta2,theta3)` — 2-link planar **plus z = theta3**, returns 4×4 pose. `L1=L2=1`.  
- `forwardKinematics_1(jointAngles)` — 3 planar links `L1=L2=L3=1` plus `z = theta4`.  
- `isSingular(jointAngles)` — loads **`mdl_puma560`**, `p560.jacob0`, singular if `|det(J)| < 1e-6`.

**Hard-coded background:**

`D:\University Data\ALL SEMESTERS\3rd Semester\Robo Tech\Assignments\Assignment #01/R2.jpg`

Replace with `fullfile` + local `R2.jpg` or the app will fail on another PC. `startupFcn` sizes `UIAxes2` very large (`[-90 -110 1700 1300]`) for that image.

Pitch deck: *Humanoid Robot Pitch Deck by Slidesgo.pptx*; report `NEW_REPORT.pdf`.

---

## How to run

MATLAB R2020b+ recommended (App Designer). Toolbox: [Peter Corke Robotics Toolbox](https://petercorke.com/toolboxes/robotics-toolbox/).

```matlab
cd('WORK_SHOPS');  run('Robot_.m');
cd('../Assignment #02');  run('ass1.m');
cd('../Assignment #01');  appdesigner('assignment_app.mlapp');
```

ESP8266 PDFs are optional hardware labs, not required to run the `.m` / `.mlapp`.

---

## Paths

Fix absolute `imread` before submission on another machine. Do not commit toolbox `PackageCache` junk.

---

## Author

Robotics coursework · [rohaan2802](https://github.com/rohaan2802)
