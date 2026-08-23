# RoboticManipulator

Robotics coursework combining **MATLAB Robotics Toolbox** workshops, quintic trajectory scripts, and a MATLAB App Designer project (`assignment_app.mlapp`) for forward/inverse kinematics, pose transforms, singularity checks, and animation.

---

## Overview

The repository is organized into:

- **WORK_SHOPS/** - guided PDFs (pose 2D/3D, complete simulation, ESP8266 demos, toolbox manuals) plus MATLAB scratch scripts (`Robot_.m`, `ws#02.m`)
- **Assignment #01/** - App Designer app, input helper `.m`, report/pitch materials, reference image
- **Assignment #02/** - quintic polynomial coefficient solve (`ass1.m`) and section PDF

`Robot_.m` builds a simple 2-link `SerialLink` ("NAO_ROB") with joint limits and `teach`/`plot`. The app text export (`assignment_app_1.txt`) shows FK helpers and a Puma 560 Jacobian singularity test via `mdl_puma560`.

---

## Features

- DH / `Link` + `SerialLink` visualization and teaching UI
- Quintic polynomial trajectory coefficient calculation between boundary conditions
- App Designer controls: Forward Kinematics, Inverse Kinematics, Transformation Matrix / POSE, Animation, Exit
- Workshop PDFs for ESP8266 and toolbox usage
- Report / pitch deck for Assignment #01

---

## Repository structure

```text
RoboticManipulator/
├── Assignment #01/
│   ├── assignment_app.mlapp
│   ├── input_values.m
│   ├── NEW_REPORT.pdf
│   ├── Humanoid Robot Pitch Deck by Slidesgo.pptx
│   └── R2.jpg
├── Assignment #02/
│   ├── ass1.m
│   └── SEC.pdf
├── WORK_SHOPS/
│   ├── Robot_.m
│   ├── ws#02.m
│   ├── qz and qr.txt
│   ├── Robotics Workshop 1 .pdf
│   ├── Workshop 2+3 Pose(2D,3D).pdf
│   ├── Week 6 - Workshop 4 - Complete Simulation*.pdf
│   ├── Tutorial Demo on ESP8266 - 1/2/3.pdf
│   └── robotics tool box manual*.pdf
└── assignment_app_1.txt          # textual export of app logic
```

---

## Build / run

### Prerequisites

- MATLAB (R2020b+ recommended for App Designer compatibility)
- [Peter Corke Robotics Toolbox for MATLAB](https://petercorke.com/toolboxes/robotics-toolbox/) on the MATLAB path
- Optional: ESP8266 hardware only if following those tutorial PDFs

### Workshop scripts

```matlab
cd('WORK_SHOPS')
run('Robot_.m')      % SerialLink teach/plot
run('ws#02.m')
```

### Quintic assignment

```matlab
cd('Assignment #02')
run('ass1.m')        % prints inverse(A)*b coefficients
```

### App Designer

```matlab
cd('Assignment #01')
appdesigner('assignment_app.mlapp')
% or:
assignment_app
```

Update any hardcoded absolute image paths inside the app (e.g. `R2.jpg` background) to your local clone path before running on another machine.

---

## Usage

1. Work through workshop PDFs in week order; reproduce plots with `Robot_.m`.
2. Use Assignment #02 to verify quintic boundary conditions analytically.
3. Launch the app for interactive FK/IK/pose demos and animation.
4. Keep `qz and qr.txt` nearby for quaternion / joint reference snippets used in labs.

---

## Extending

- Replace absolute `imread` paths with relative `fullfile(fileparts(mfilename('fullpath')),'R2.jpg')`.
- Add a third workshop script that loads measured joint trajectories from CSV.
- Export App Designer callbacks cleanly into `.m` classes for version control diffs.
- Bridge ESP8266 tutorials with serial/UDP joint streaming into `SerialLink.plot`.

---

## License

Academic robotics materials - Robotics Toolbox and Slidesgo decks follow their respective licenses.
