# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

RunnymedeSwerve is a Java library for FRC (FIRST Robotics Competition) swerve drive systems. It provides an abstraction layer over complex swerve drive logic, designed to be accessible to high school robotics teams while remaining educational and extensible.

## Development Commands

### Build and Test
```bash
# Clean and build the project
./gradlew clean build

# Run tests
./gradlew test

# Run a single test
./gradlew test --tests "TestClassName"
```

### Local Development
```bash
# Build and publish to local Maven repository for development
./gradlew clean && \
./gradlew build && \
./gradlew publishToMavenLocal
```

### Publishing (Maintainers Only)
```bash
# First-time publish
./gradlew clean && \
./gradlew publish && \
./gradlew jreleaserInit && \
./gradlew jreleaserDeploy

# Subsequent publishes
./gradlew clean && \
./gradlew publish && \
./gradlew jreleaserDeploy
```

### Code Formatting
This library uses prettier-java for code formatting. Format code before committing.

## Architecture Overview

### Core Interface
- `RunnymedeSwerveDrive` - Main interface defining swerve drive contract
- Provides methods for driving (`drive()`), locking, odometry, and gyro operations

### Implementation Hierarchy
```
RunnymedeSwerveDrive (interface)
└── CoreSwerveDrive (base implementation)
    └── GyroAwareSwerveDrive (adds gyro support)
        └── FieldAwareSwerveDrive (adds field-relative odometry)
            └── LimelightAwareSwerveDrive (adds vision pose estimation)
```

### Key Components

**Core Package (`ca.team1310.swerve.core`)**
- `CoreSwerveDrive` - Base swerve drive implementation with threading
- `SwerveModule` / `SwerveModuleImpl` - Individual swerve module control
- `SwerveMath` - Kinematics and mathematical utilities
- `ModuleState` / `ModuleDirective` - Module state management
- `DriveMotor` / `AngleMotor` - Motor abstractions
- `AbsoluteAngleEncoder` - Encoder interface

**Hardware Support (`core.hardware`)**
- `rev.neospark.*` - REV Robotics NEO/Spark motor controllers
- `cancoder.*` - CANCoder absolute encoders

**Configuration (`core.config`)**
- `CoreSwerveConfig` - Primary configuration class
- Hardware-specific configuration classes

**Odometry Package (`odometry`)**
- `FieldAwareSwerveDrive` - Field-relative positioning using WPILib's `SwerveDrivePoseEstimator`

**Vision Package (`vision`)**
- `LimelightAwareSwerveDrive` - Integrates Limelight vision for pose estimation
- `VisionConfig` - Vision system configuration

**Gyro Package (`gyro`)**
- `GyroAwareSwerveDrive` - Gyro integration base class
- Hardware-specific gyro implementations

**Threading Model**
- Separate threads for module updates (5ms period) and sensor reads (5ms period)
- Odometry updates every 10ms, telemetry every 250ms
- Uses WPILib `Notifier` for thread management

### Usage Pattern
1. Configure `CoreSwerveConfig` with hardware specifications
2. Instantiate appropriate drive implementation (`FieldAwareSwerveDrive` or `LimelightAwareSwerveDrive`)
3. Call `drive(vx, vy, omega)` from robot subsystem
4. Use odometry methods (`getPose()`, `resetOdometry()`) as needed

### Key Design Principles
- Educational: Code designed to be understandable by high school students
- Hardware Abstraction: Support multiple motor controller and encoder types
- Thread Safety: Internal threading handles periodic operations
- Field Coordinates: Uses standard FRC field coordinate system (positive X forward, positive Y left)