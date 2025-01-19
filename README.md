# Runnymede Swerve

This library provides an easy-to-use abstraction over the core logic of a swerve drive. It is designed to be used with 
FRC robots. This drive currently includes two main implementations: A `FieldAwareSwerveDrive`, which uses odometry to
track the position of the robot on the field, and a `VisionAwareSwerveDrive`, which uses odometry and vision measurements
to track the position of the robot on the field.

The primary interface, `RunnymedeSwerve`, provides a brief set of functions that allow the user to control the swerve 
drive. It is designed to be used in a drive subsystem where the user can call `drive` to control the robot.

The current implementation only supports limited hardware configuration. Additional hardware configurations can be added
when requested.

This library was heavily inspired by the [FRC Swerve Library]() and the [YAGSL]() libraries, but simplified to make it
accessible to high school students.

It is designed to allow most FRC team members to be able to interact with it, and senior high school students to be able
to understand the code contained within it. With the exception of some `edu.wpilib` classes like `SwerveDriveKinematics`,
most of the code is written in a way that is easy to understand.

## Installation

To use this library, add the following to your `build.gradle` file:

```gradle
    implementation 'ca.team1310:swerve:2.0.6'
```

Then, configure the `CoreSwerveConfig` class to construct your swerve drive. If you are using `VisionAwareSwerveDrive`,
you will also need to configure the `VisionConfig` class.

## Usage

There is no need to call any methods during your subsystem's `periodic()` method, as  `RunnymedeSwerve` spints up
its own threads to perform all relevant periodic calculations.

```java
...
public class SwerveSubsystem extends SubsystemBase {

    ...
    private final RunnymedeSwerveDrive drive;
    private final SubsystemConfig subsystemConfig;
    private final CoreSwerveConfig coreConfig;

    ...

    public SwerveSubsystem(SubsytemConfig subsystemConfig, CoreSwerveConfig coreConfig) {
        this.drive = new FieldAwareSwerveDrive(coreConfig);
        ...
    }

    private void driveSafely(ChassisSpeeds robotOrientedVelocity) {
        double x = robotOrientedVelocity.vxMetersPerSecond;
        double y = robotOrientedVelocity.vyMetersPerSecond;
        double w = robotOrientedVelocity.omegaRadiansPerSecond;

        // Limit change in values. Note this may not scale
        // evenly - one may reach desired speed before another.

        // Use driveFieldOriented to avoid this.

        x = xLimiter.calculate(x);
        y = yLimiter.calculate(y);
        w = omegaLimiter.calculate(w);

        ChassisSpeeds safeVelocity = new ChassisSpeeds(x, y, w);

        if (this.config.enabled()) {
            this.drive.drive(safeVelocity);
        }
    }
    ...
}

```





---
# Enhancing This Library (Advanced Users)
## Local Development

If you are actively developing this library, or require a local build for some reason, you can build the library
locally.

```bash
./gradlew clean && \
./gradlew build && \
./gradlew publishToMavenLocal
```

This will publish the library to your local maven repository, which you can then reference in your project. However,
your project must include the `mavenLocal()` repository. WPILIB projects do not include this by default (they look for a
maven repository in a different location).

_### Code Formatting
This library uses prettier java code formatting. [Be sure to follow the instructions here](https://github.com/jhipster/prettier-java/blob/main/docs/advanced_usage.md)._

## Publishing Maven Package

You need to make a `gradle.properties` in the base directory, containing:

```
gpr.user=<github username>
gpr.token=<private access token>
```

To obtain a private access token, go to [Tokens Classic](https://github.com/settings/tokens) and create a new Token with
`write:packages` and `read:packages` permissions.

*Do Not Check This File In*

## To Publish the First Time

We don't really know exactly what is required here yet but this seems to work

```bash
./gradlew clean && \
./gradlew publish && \
./gradlew jreleaserInit && \
./gradlew jreleaserDeploy
```

## To Publish the Second or Later Time:

```bash
./gradlew clean && \
./gradlew publish && \
./gradlew jreleaserDeploy
```
