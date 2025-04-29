package ca.team1310.swerve.core;

import static ca.team1310.swerve.core.config.TelemetryLevel.*;

import ca.team1310.swerve.RunnymedeSwerveDrive;
import ca.team1310.swerve.SwerveTelemetry;
import ca.team1310.swerve.core.config.CoreSwerveConfig;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotBase;

/** The Core swerve drive object. This implements most core features of a swerve drive */
public class CoreSwerveDrive implements RunnymedeSwerveDrive {

  /*
   * Module array order - note, these are aligned with SwerveMath order for simplicity.
   * <code>
   *     1 ------ 0
   *     |        |
   *     |        |
   *     2 ------ 3
   * </code>
   */
  private final SwerveModule[] modules;
  private final ModuleState[] moduleStates;

  private final SwerveMath math;

  private double desiredVx;
  private double desiredVy;
  private double desiredOmega;
  private final double MINIMUM_OMEGA_VALUE_RAD_PER_SEC = Math.toRadians(1);

  /** The telemetry object for the swerve drive */
  protected final SwerveTelemetry telemetry;

  /** Whether the robot is running in simulation mode */
  protected final boolean isSimulation;

  //
  // Thread controls
  //
  // module update thread
  private final Notifier updateModuleThread = new Notifier(this::updateModules);
  private static final int UPDATE_MODULES_PERIOD = 5; // milliseconds

  // module state refresher thread
  private final Notifier readModulesThread = new Notifier(this::readModuleStates);
  private static final int READ_MODULES_PERIOD_MILLIS = 5; // milliseconds
  private static final int ODOMETRY_UPDATE_CYCLES = 2;
  private static final int TELEMETRY_UPDATE_CYCLES = 50;
  private int moduleStateUpdateCount = 0;

  private final Notifier telemetryThread = new Notifier(this::updateTelemetry);
  private static final int TELEMETRY_UPDATE_PERIOD = 20; // milliseconds

  /**
   * Construct a new CoreSwerveDrive object with the specified configuration.
   *
   * @param cfg the configuration of the swerve drive
   */
  protected CoreSwerveDrive(CoreSwerveConfig cfg) {
    System.out.println("Initializing RunnymedeSwerve CoreSwerveDrive.");
    // order matters in case we want to use AdvantageScope
    double dt = cfg.robotPeriodSeconds();
    this.modules = new SwerveModule[4];
    this.isSimulation = RobotBase.isSimulation();
    if (isSimulation) {
      this.modules[0] = new SwerveModuleSimulation(cfg.frontRightModuleConfig());
      this.modules[1] = new SwerveModuleSimulation(cfg.frontLeftModuleConfig());
      this.modules[2] = new SwerveModuleSimulation(cfg.backLeftModuleConfig());
      this.modules[3] = new SwerveModuleSimulation(cfg.backRightModuleConfig());
    } else {
      this.modules[0] =
          new SwerveModuleImpl(
              cfg.frontRightModuleConfig(),
              cfg.maxAttainableModuleSpeedMetresPerSecond(),
              (int) (dt * 1000));
      this.modules[1] =
          new SwerveModuleImpl(
              cfg.frontLeftModuleConfig(),
              cfg.maxAttainableModuleSpeedMetresPerSecond(),
              (int) (dt * 1000));
      this.modules[2] =
          new SwerveModuleImpl(
              cfg.backLeftModuleConfig(),
              cfg.maxAttainableModuleSpeedMetresPerSecond(),
              (int) (dt * 1000));
      this.modules[3] =
          new SwerveModuleImpl(
              cfg.backRightModuleConfig(),
              cfg.maxAttainableModuleSpeedMetresPerSecond(),
              (int) (dt * 1000));
    }

    this.moduleStates = new ModuleState[4];
    this.moduleStates[0] = modules[0].getState();
    this.moduleStates[1] = modules[1].getState();
    this.moduleStates[2] = modules[2].getState();
    this.moduleStates[3] = modules[3].getState();

    this.math =
        new SwerveMath(
            cfg.wheelBaseMetres(),
            cfg.trackWidthMetres(),
            cfg.maxAttainableTranslationSpeedMetresPerSecond(),
            cfg.maxAchievableRotationalVelocityRadiansPerSecond());

    this.telemetry = new SwerveTelemetry(4);
    this.telemetry.level = cfg.telemetryLevel();
    this.telemetry.maxModuleSpeedMPS = cfg.maxAttainableModuleSpeedMetresPerSecond();
    this.telemetry.maxTranslationSpeedMPS = cfg.maxAttainableTranslationSpeedMetresPerSecond();
    this.telemetry.maxRotationalVelocityRadPS =
        cfg.maxAchievableRotationalVelocityRadiansPerSecond();
    this.telemetry.trackWidthMetres = cfg.trackWidthMetres();
    this.telemetry.wheelBaseMetres = cfg.wheelBaseMetres();
    this.telemetry.wheelRadiusMetres = cfg.wheelRadiusMetres();
    this.telemetry.moduleNames[0] = cfg.frontRightModuleConfig().name();
    this.telemetry.moduleNames[1] = cfg.frontLeftModuleConfig().name();
    this.telemetry.moduleNames[2] = cfg.backLeftModuleConfig().name();
    this.telemetry.moduleNames[3] = cfg.backRightModuleConfig().name();
    this.telemetry.moduleWheelLocations[0] = cfg.frontRightModuleConfig().location().getX();
    this.telemetry.moduleWheelLocations[1] = cfg.frontRightModuleConfig().location().getY();
    this.telemetry.moduleWheelLocations[2] = cfg.frontLeftModuleConfig().location().getX();
    this.telemetry.moduleWheelLocations[3] = cfg.frontLeftModuleConfig().location().getY();
    this.telemetry.moduleWheelLocations[4] = cfg.backLeftModuleConfig().location().getX();
    this.telemetry.moduleWheelLocations[5] = cfg.backLeftModuleConfig().location().getY();
    this.telemetry.moduleWheelLocations[6] = cfg.backRightModuleConfig().location().getX();
    this.telemetry.moduleWheelLocations[7] = cfg.backRightModuleConfig().location().getY();

    // start up threads
    updateModuleThread.setName("RunnymedeSwerve updateModules");
    updateModuleThread.startPeriodic(UPDATE_MODULES_PERIOD / 1000.0);

    readModulesThread.setName("RunnymedeSwerve readModuleStates");
    readModulesThread.startPeriodic(READ_MODULES_PERIOD_MILLIS / 1000.0);

    telemetryThread.setName("RunnymedeSwerve updateTelemetry");
    telemetryThread.startPeriodic(
        isSimulation ? READ_MODULES_PERIOD_MILLIS / 1000.0 : TELEMETRY_UPDATE_PERIOD / 1000.0);
  }

  public final synchronized void drive(double x, double y, double w) {
    desiredVx = x;
    desiredVy = y;
    desiredOmega = Math.abs(w) < MINIMUM_OMEGA_VALUE_RAD_PER_SEC ? 0 : w;
  }

  /*
   * Set the module states based on the desired speed and angle.
   */
  private synchronized void updateModules() {
    // calculate desired states
    math.calculateAndStoreModuleVelocities(desiredVx, desiredVy, desiredOmega);

    // set the module states
    this.modules[0].setDesiredState(math.getFrontRight());
    this.modules[1].setDesiredState(math.getFrontLeft());
    this.modules[2].setDesiredState(math.getBackLeft());
    this.modules[3].setDesiredState(math.getBackRight());

    if (isSimulation) {
      updateGyroForSimulation();
    }
  }

  /** Update the gyro in case the robot is running in simulation mode. */
  protected void updateGyroForSimulation() {}

  private synchronized void readModuleStates() {
    boolean odometry = moduleStateUpdateCount % ODOMETRY_UPDATE_CYCLES == 0;
    boolean telemetry = moduleStateUpdateCount % TELEMETRY_UPDATE_CYCLES == 0;
    if (moduleStateUpdateCount > 10000) {
      moduleStateUpdateCount = 0;
    } else {
      moduleStateUpdateCount++;
    }
    this.modules[0].readState(odometry, telemetry);
    this.modules[1].readState(odometry, telemetry);
    this.modules[2].readState(odometry, telemetry);
    this.modules[3].readState(odometry, telemetry);
  }

  /**
   * Get the states of the swerve modules, including their speed and angle.
   *
   * @return an array of swerve module states in the order front left, front right, back left, back
   *     right.
   */
  protected ModuleState[] getModuleStates() {
    return moduleStates;
  }

  @Override
  public synchronized void setModuleState(String moduleName, double speed, double angle) {
    // This is for TEST MODE ONLY!!! Not for internal use or drive use.
    for (SwerveModule module : modules) {
      if (module.getName().equals(moduleName)) {
        ModuleDirective desiredState = new ModuleDirective();
        desiredState.set(speed, angle);
        module.setDesiredState(desiredState);
        break;
      }
    }
    this.desiredVx = 0;
    this.desiredVy = 0;
    this.desiredOmega = 0;
  }

  /**
   * Get the measured velocity of the robot, given the current module states.
   *
   * @return an array of the robot's velocity in the form [vx, vy, omega], where vx and vy are in
   *     m/s and omega is in rad/s (counter-clockwise is positive).
   */
  public double[] getMeasuredRobotVelocity() {
    return math.calculateRobotVelocity(
        moduleStates[0].getSpeed(),
        Math.toRadians(moduleStates[0].getAngle()),
        moduleStates[1].getSpeed(),
        Math.toRadians(moduleStates[1].getAngle()),
        moduleStates[2].getSpeed(),
        Math.toRadians(moduleStates[2].getAngle()),
        moduleStates[3].getSpeed(),
        Math.toRadians(moduleStates[3].getAngle()));
  }

  @Override
  public final synchronized boolean lock() {
    boolean moving = false;

    // safety code to prevent locking if robot is moving
    for (SwerveModule swerveModule : modules) {
      // do not lock if moving more than 4cm/s
      if (Math.abs(swerveModule.getState().getSpeed()) > 0.04) {
        moving = true;
      }
    }

    if (!moving) {
      desiredVx = 0;
      desiredVy = 0;
      desiredOmega = 0;

      // set speed to 0 and angle wheels to center
      for (SwerveModule module : modules) {
        ModuleDirective lockState = new ModuleDirective();
        lockState.set(0, 0);
        module.setDesiredState(lockState);
      }
    }

    return !moving;
  }

  private synchronized void updateTelemetry() {
    updateTelemetry(this.telemetry);
  }

  /**
   * Update the telemetry of the swerve drive, using data from the drivebase.
   *
   * <p>Overriding methods should add their own data to telemetry and then call
   * super.updateTelemetry(telemetry) to ensure all data is collected and reported consistently.
   *
   * <p>The frequency at which this is called depends on how frequently the core swerve drive calls
   * it. This is not currently configurable.
   *
   * @param telemetry the telemetry object to update
   */
  protected synchronized void updateTelemetry(SwerveTelemetry telemetry) {
    if (telemetry.level == INPUT || telemetry.level == CALCULATED || telemetry.level == VERBOSE) {
      telemetry.desiredChassisSpeeds[0] = this.desiredVx;
      telemetry.desiredChassisSpeeds[1] = this.desiredVy;
      telemetry.desiredChassisSpeeds[2] = Math.toDegrees(this.desiredOmega);

      if (telemetry.level == CALCULATED || telemetry.level == VERBOSE) {
        var measuredChassisSpeeds = getMeasuredRobotVelocity();
        telemetry.measuredChassisSpeeds[0] = measuredChassisSpeeds[0];
        telemetry.measuredChassisSpeeds[1] = measuredChassisSpeeds[1];
        telemetry.measuredChassisSpeeds[2] = Math.toDegrees(measuredChassisSpeeds[2]);
      }
    }

    for (int i = 0; i < modules.length; i++) {
      modules[i].checkFaults();

      ModuleState state = modules[i].getState();

      if (telemetry.level == INPUT || telemetry.level == CALCULATED || telemetry.level == VERBOSE) {
        // desired states
        telemetry.moduleDesiredStates[i * 2] = Math.toRadians(state.getDesiredAngle());
        telemetry.moduleDesiredStates[i * 2 + 1] = state.getDesiredSpeed();
      }

      if (telemetry.level == CALCULATED || telemetry.level == VERBOSE) {
        // measured states
        telemetry.moduleMeasuredStates[i * 2] = Math.toRadians(state.getAngle());
        telemetry.moduleMeasuredStates[i * 2 + 1] = state.getSpeed();

        // location information
        telemetry.moduleAngleMotorPositionDegrees[i] = state.getAngle();
        telemetry.moduleDriveMotorPositionMetres[i] = state.getPosition();
      }

      if (telemetry.level == VERBOSE) {
        telemetry.driveMotorOutputPower[i] = state.getDriveOutputPower();
        // angle encoder
        telemetry.moduleAbsoluteEncoderPositionDegrees[i] = state.getAbsoluteEncoderAngle();
      }
    }
    // post it!
    telemetry.post();
  }
}
