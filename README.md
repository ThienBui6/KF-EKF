# KF-EKF
├── include/
│   └── TrackingFilters.hpp  # Filter implementations
├── src/
│   └── main.cpp             # Usage example
├── CMakeLists.txt
└── README.md

This README includes:
1. Contextual alignment with the original paper
2. Installation/usage instructions
3. Key customization points matching paper's technical content
4. Clear code structure explanation
5. Required dependencies
6. Academic references
7. License and contribution guidelines

The implementation can be extended with additional features from the paper like:
- JPDA/MHT association algorithms
- OSPA/GOSPA metrics
- Full SGPA perturbation model
- Radar FoV constraints

- # Space Surveillance Tracking Filters

C++ implementation of Kalman Filter (KF) and Extended Kalman Filter (EKF) for Low Earth Orbit (LEO) object tracking, based on the concepts presented in "Performance Analysis of Space Surveillance and Tracking Systems".

## Features

- **Two Filter Implementations**:
  - Linear Kalman Filter (KF) with constant velocity model
  - Extended Kalman Filter (EKF) with orbital dynamics (two-body problem)
  
- **Key Components**:
  - State prediction with process noise (Q matrix)
  - Measurement update with sensor noise (R matrix)
  - Simplified orbital dynamics model (Equation 3)
  - ECEF coordinate system handling
  - Eigen-based matrix operations

- **Paper-Aligned Features**:
  - Mono-static radar measurement model (Section 2.1)
  - Basic track maintenance logic framework
  - Configurable noise parameters (Tables 2,3,7)

## Requirements

- C++17 compatible compiler
- Eigen3 library (linear algebra)
- CMake (recommended)

## Installation

1. Install Eigen:
   ```bash
   # Ubuntu/Debian
   sudo apt-get install libeigen3-dev
   
   # macOS
   brew install eigen
