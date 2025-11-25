# UR10e Robot Simulator

A comprehensive MATLAB-based simulator for the Universal Robots UR10e collaborative robot, featuring forward/inverse kinematics, velocity analysis, singularity detection, and trajectory planning.

![UR10e Simulator](https://img.shields.io/badge/MATLAB-R2020a+-blue.svg)
![License](https://img.shields.io/badge/license-MIT-green.svg)

## 🌟 Features

### Core Functionality
- **Forward Kinematics (FK)**: Real-time end-effector pose calculation using DH parameters
- **Inverse Kinematics (IK)**: Damped Least-Squares solver with workspace validation
- **Velocity Kinematics**: Jacobian-based end-effector velocity computation
- **Singularity Detection**: Automatic detection and classification of robot singularities
- **3D Visualization**: URDF-based robot rendering with interactive controls

### Trajectory Planning
Four pre-programmed trajectory patterns:
- **Circle Path**: Circular motion in 3D space
- **Square Path**: Rectangular path following
- **Line Path**: Point-to-point linear motion
- **Figure-8 Path**: Lemniscate trajectory

### Advanced Features
- **Workspace Visualization**: Graphical representation of reachable workspace
- **Real-time Singularity Monitoring**: Visual alerts for singular configurations
- **Interactive GUI**: Slider-based joint control with numerical input
- **Path Visualization**: Real-time end-effector trajectory display

## 📋 Requirements

### Software Dependencies
- MATLAB R2020a or later
- Robotics System Toolbox
- Peter Corke's Robotics Toolbox for MATLAB(IF YOU DO NOT WANT TO USE MY OWN CODE YOU CAN DOWNLOAD THE PERTER CORKE'S TOOLBOX)
  ```matlab
  % Install from: https://petercorke.com/toolboxes/robotics-toolbox/
  ```

### Hardware Requirements
- Minimum 8GB RAM
- Graphics card supporting OpenGL 3.3+

## 🚀 Installation

1. **Clone the repository**
   ```bash
   git clone https://github.com/yourusername/ur10e-simulator.git
   cd ur10e-simulator
   ```

2. **Install Peter Corke's Robotics Toolbox**
   - Download from [Peter Corke's website](https://petercorke.com/toolboxes/robotics-toolbox/)
   - Add to MATLAB path:
     ```matlab
     addpath('path/to/robotics-toolbox')
     startup_rvc
     ```

3. **Add required files**
   - Place `ur10e.urdf` in the project directory
   - Place `BK_LOGO.png` (optional, for branding)

4. **Run the simulator**
   ```matlab
   main_extended
   ```

## 📖 Usage Guide

### Basic Operation

#### 1. Forward Kinematics
- Adjust joint angle sliders (J1-J6) to move the robot
- Real-time position and orientation display
- Singularity status indicator

#### 2. Inverse Kinematics
- Enter desired end-effector position (X, Y, Z) in meters
- Enter orientation (Roll, Pitch, Yaw) in radians
- Click "Solve IK & Apply" to compute joint angles
- Automatic workspace boundary checking

#### 3. Velocity Kinematics
- Enter joint velocities (q1-q6) in rad/s
- Click "Compute Velocity" to calculate end-effector velocity
- Displays linear and angular velocity components
- Shows Jacobian determinant for singularity analysis

#### 4. Trajectory Execution
- Click trajectory buttons (Circle, Square, Line, Figure-8)
- Watch real-time animated motion
- Red path shows end-effector trajectory
- Smooth interpolation using PCHIP method

### Workspace Limits

```
Maximum Reach: ~1.184 m (0.613 + 0.571)
Minimum Reach: ~0.10 m
Z Range: [-0.3, 1.3] m
```

### DH Parameters

The simulator uses the following Denavit-Hartenberg parameters for UR10e:

| Link | a (m)   | α (rad) | d (m)  | θ (rad) |
|------|---------|---------|--------|---------|
| 1    | 0       | π/2     | 0.181  | θ₁      |
| 2    | -0.613  | -π      | 0      | θ₂      |
| 3    | -0.571  | π       | 0      | θ₃      |
| 4    | 0       | -π/2    | 0      | θ₄      |
| 5    | 0       | π/2     | 0      | θ₅      |
| 6    | -0.12   | 0       | 0.117  | θ₆      |

## 🔧 Code Structure

```
ur10e-simulator/
├── main_extended.m           # Main program file
├── ur10e.urdf               # Robot URDF model
├── BK_LOGO.png              # Optional logo image
└── README.md                # This file
```

### Key Functions

#### Core Kinematics
- `FK(DH)` - Forward kinematics calculation
- `IK_UR10e(T_des)` - Inverse kinematics solver
- `computeNumericJacobian(DH_params)` - Jacobian matrix computation

#### Singularity Analysis
- `detectSingularity(DH_params, threshold)` - Singularity detection
  - Wrist singularity (θ₅ ≈ 0 or π)
  - Elbow singularity (θ₃ ≈ 0 or π)
  - Shoulder/boundary singularity

#### Trajectory Generation
- `generateTrajectory(waypoints_cart, waypoints_rpy, ...)` - Smooth trajectory planning
- `animateTrajectory(robot, q_trajectory, ...)` - Real-time animation

#### Trajectory Patterns
- `executeCirclePath()` - Circular motion
- `executeSquarePath()` - Square path
- `executeLinePath()` - Linear interpolation
- `executeFigure8Path()` - Figure-8 pattern

## 📊 Examples

### Example 1: IK Solver
```matlab
% Target position
X = 0.7, Y = 0.2, Z = 0.5

% Target orientation (RPY)
Roll = 0, Pitch = π/2, Yaw = 0

% The solver will:
% 1. Check workspace reachability
% 2. Compute joint angles
% 3. Verify solution accuracy
% 4. Update robot visualization
```

### Example 2: Velocity Analysis
```matlab
% Joint velocities (rad/s)
q_dot = [0.1, 0.0, 0.0, 0.0, 0.0, 0.0]

% Output:
% - Linear velocity: [vx, vy, vz] m/s
% - Angular velocity: [wx, wy, wz] rad/s
% - Jacobian determinant
```

## ⚠️ Singularity Types

The simulator detects three main singularity types:

1. **Wrist Singularity**
   - Occurs when θ₅ ≈ 0 or π
   - Axes 4 and 6 become collinear
   - Loss of orientation control

2. **Elbow Singularity**
   - Occurs when θ₃ ≈ 0 or π
   - Arm fully extended or folded
   - Limited workspace mobility

3. **Shoulder/Boundary Singularity**
   - Occurs at workspace boundaries
   - Multiple joints align
   - Complex motion required

## 🐛 Troubleshooting

### Common Issues

**Issue**: "Failed to load robot"
- **Solution**: Ensure `ur10e.urdf` is in the working directory

**Issue**: "IK failed! No solution found"
- **Solution**: Check if target is within workspace limits

**Issue**: Logo not displaying
- **Solution**: Verify `BK_LOGO.png` exists or comment out logo code

**Issue**: Trajectory animation is slow
- **Solution**: Reduce number of waypoints in trajectory functions

## 🔬 Technical Details

### IK Solver Algorithm
- Method: Damped Least-Squares (Levenberg-Marquardt)
- Max iterations: 800
- Position tolerance: 0.0001 m
- Orientation tolerance: 0.001 rad
- Damping factor: 0.01
- Max step size: 0.2 rad

### Trajectory Interpolation
- Method: Piecewise Cubic Hermite Interpolation (PCHIP)
- Segment time: 1.0 second
- Sample rate: 0.05 seconds (20 Hz)
- Smooth joint-space transitions

## 📝 License

This project is licensed under the MIT License - see below for details:

```
MIT License

Copyright (c) 2025

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
```

## 👥 Contributing

Contributions are welcome! Please follow these steps:

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/AmazingFeature`)
3. Commit your changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

## 📧 Contact

Project Maintainer: [Hero]
- Email: hungnguyentranduc266@gmail.com
- GitHub: [Hug-leo](https://github.com/Hug-leo)

## 🙏 Acknowledgments

- Peter Corke for the Robotics Toolbox
- Universal Robots for UR10e specifications
- Bach Khoa University (BK) for academic support

## 📚 References

1. Robot Modeling and Control-Spong
2. Peter Corke's Robotics Toolbox Documentation

---

**Note**: This simulator is for educational and research purposes. Always follow proper safety protocols when working with actual robotic hardware.

⭐ If you find this project useful, please consider giving it a star on GitHub!