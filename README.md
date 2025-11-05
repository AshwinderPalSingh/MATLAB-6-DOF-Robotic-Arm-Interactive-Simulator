# MATLAB 6-DOF Robotic Arm Interactive Simulator

This repository contains a MATLAB implementation for the forward kinematics of a 6-DOF (Degrees of Freedom) robotic arm, complete with an interactive GUI.

The kinematics are calculated based on the Denavit-Hartenberg (D-H) convention. The project is structured to be modular, separating the GUI from the core kinematics functions.

## Features

- **Interactive GUI**: Control all 6 joint angles using sliders.
- **Real-Time Visualization**: The 3D plot of the arm updates instantly.
- **Live Kinematics**: The (X, Y, Z) position of the end-effector is calculated and displayed in real-time.
- **Modular Design**: The core math functions (`functions` folder) are completely separate from the GUI, making them reusable for other projects (e.g., inverse kinematics, trajectory planning).

## Project Structure

```
.
├── functions
│   ├── compute_fk.m
│   ├── define_6dof_robot.m
│   └── dh_transform.m
├── interactive_6DOF_simulator.m
└── README.md
```


- **`interactive_6DOF_simulator.m`**: (The Main File) Run this script to launch the GUI.
- **`/functions`**: A folder containing all the core reusable functions.
  - `define_6dof_robot.m`: Returns a struct with the 6-DOF arm's D-H parameters.
  - `dh_transform.m`: Utility function to compute a single D-H transformation matrix.
  - `compute_fk.m`: Core N-DOF forward kinematics solver.

## How to Run

1. Ensure the `functions` folder and all its contents are in the same directory as `interactive_6DOF_simulator.m`.
2. Open `interactive_6DOF_simulator.m` in MATLAB.
3. Click the **Run** button.
4. The GUI will appear. Move the sliders to control the robot.
