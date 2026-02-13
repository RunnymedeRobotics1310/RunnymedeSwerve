package ca.team1310.swerve.core;

import static ca.team1310.swerve.core.config.TelemetryLevel.*;

import ca.team1310.swerve.RunnymedeSwerveDrive;
import ca.team1310.swerve.SwerveTelemetry;
import ca.team1310.swerve.core.config.CoreSwerveConfig;
import ca.team1310.swerve.utils.SwerveUtils;
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

  private final SwerveKinematics kinematics;

  protected double desiredVx;
  protected double desiredVy;
  protected double desiredOmega;
  protected final double MINIMUM_OMEGA_VALUE_RAD_PER_SEC = Math.toRadians(1);

  /** The telemetry object for the swerve drive */
  protected final SwerveTelemetry telemetry;

  /** Whether the robot is running in simulation mode */
  protected final boolean isSimulation;

  //
  // Thread controls
  //
  /**
   * This is the period (in milliseconds) at which modules are read and updated during driving
   * operations. We typically want this to be as quick as possible, but need to monitor CAN
   * utilization. Setting this to 10ms vs 20ms has no impact on CAN utilization. Setting it to 5ms
   * vs 10ms has a significant impact (on a Roborio1 with default CAN, utilization went from 30% to
   * 50% when going from 10ms to 5ms).
   */
  public static final int MANAGE_MODULES_PERIOD_MS = 10;

  private final Notifier moduleManagementThread = new Notifier(this::updateModules);

  private static final int SYNC_ENCODERS_PERIOD_MS = 500;
  private final Notifier encoderSyncThread = new Notifier(this::syncAllEncoders);

  public static final int TELEMETRY_UPDATE_PERIOD_MS = 50; // milliseconds
  private final Notifier telemetryThread = new Notifier(this::updateTelemetry);

  /**
   * Construct a new CoreSwerveDrive object with the specified configuration.
   *
   * @param cfg the configuration of the swerve drive
   */
  protected CoreSwerveDrive(CoreSwerveConfig cfg) {
    System.out.println("Initializing RunnymedeSwerve.");
    System.out.println("Swerve module update period: " + MANAGE_MODULES_PERIOD_MS + " ms");
    System.out.println("Swerve encoder sync period: " + SYNC_ENCODERS_PERIOD_MS + " ms");
    System.out.println("Swerve telemetry update period: " + TELEMETRY_UPDATE_PERIOD_MS + " ms");

    // order matters in case we want to use AdvantageScope
    this.modules = new SwerveModule[4];
    final double maxModSpdMps = cfg.maxAttainableModuleSpeedMetresPerSecond();
    this.isSimulation = RobotBase.isSimulation();
    if (isSimulation) {
      this.modules[0] = new SwerveModuleSimulation(cfg.frontRightModuleConfig(), maxModSpdMps);
      this.modules[1] = new SwerveModuleSimulation(cfg.frontLeftModuleConfig(), maxModSpdMps);
      this.modules[2] = new SwerveModuleSimulation(cfg.backLeftModuleConfig(), maxModSpdMps);
      this.modules[3] = new SwerveModuleSimulation(cfg.backRightModuleConfig(), maxModSpdMps);
    } else {
      this.modules[0] = new SwerveModuleImpl(cfg.frontRightModuleConfig(), maxModSpdMps);
      this.modules[1] = new SwerveModuleImpl(cfg.frontLeftModuleConfig(), maxModSpdMps);
      this.modules[2] = new SwerveModuleImpl(cfg.backLeftModuleConfig(), maxModSpdMps);
      this.modules[3] = new SwerveModuleImpl(cfg.backRightModuleConfig(), maxModSpdMps);
    }

    this.moduleStates = new ModuleState[4];
    this.moduleStates[0] = modules[0].getState();
    this.moduleStates[1] = modules[1].getState();
    this.moduleStates[2] = modules[2].getState();
    this.moduleStates[3] = modules[3].getState();

    this.kinematics =
        new SwerveKinematics(
            cfg.wheelBaseMetres(),
            cfg.trackWidthMetres(),
            maxModSpdMps,
            cfg.maxAchievableRotationalVelocityRadiansPerSecond(),
            cfg.discretizeTransScale(),
            cfg.discretizeRotScale(),
            cfg.discretizeNormalScale());

    this.telemetry = new SwerveTelemetry(4);
    this.telemetry.level = cfg.telemetryLevel();
    this.telemetry.maxModuleSpeedMPS = maxModSpdMps;
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
    moduleManagementThread.setName("RunnymedeSwerve manageModuleStates");
    moduleManagementThread.startPeriodic(MANAGE_MODULES_PERIOD_MS / 1000.0);

    encoderSyncThread.setName("RunnymedeSwerve syncEncoders");
    encoderSyncThread.startPeriodic(SYNC_ENCODERS_PERIOD_MS / 1000.0);

    telemetryThread.setName("RunnymedeSwerve updateTelemetry");
    // in simulation mode, provide telemetry faster but while driving use slower rate
    telemetryThread.startPeriodic(isSimulation ? .02 : TELEMETRY_UPDATE_PERIOD_MS / 1000.0);
  }

  public synchronized void driveRobotOriented(double x, double y, double w) {

    // if below minimum speed just stop rotating
    w = Math.abs(w) < MINIMUM_OMEGA_VALUE_RAD_PER_SEC ? 0 : w;

    // set desired speeds
    desiredVx = x;
    desiredVy = y;
    desiredOmega = w;
  }

  /*
   * Set the module states based on the desired speed and angle.
   */
  protected synchronized void updateModules() {

    // read module states
    for (SwerveModule module : modules) {
      module.readState();
    }

    // calculate desired states
    kinematics.calculateModuleVelocities(desiredVx, desiredVy, desiredOmega);

    // set the module states
    this.modules[0].setDesiredState(kinematics.getFrontRight());
    this.modules[1].setDesiredState(kinematics.getFrontLeft());
    this.modules[2].setDesiredState(kinematics.getBackLeft());
    this.modules[3].setDesiredState(kinematics.getBackRight());

    if (isSimulation) {
      updateGyroForSimulation();
    }
  }

  /** Sync relative encoders toward absolute encoders on a separate, slower thread. */
  private synchronized void syncAllEncoders() {
    for (SwerveModule module : modules) {
      module.syncEncoders();
    }
  }

  /** Update the gyro in case the robot is running in simulation mode. */
  protected void updateGyroForSimulation() {}

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
    return kinematics.calculateRobotVelocity(
        moduleStates[0].getSpeed(),
        moduleStates[0].getAngle(),
        moduleStates[1].getSpeed(),
        moduleStates[1].getAngle(),
        moduleStates[2].getSpeed(),
        moduleStates[2].getAngle(),
        moduleStates[3].getSpeed(),
        moduleStates[3].getAngle());
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
    for (SwerveModule module : modules) {
      module.readVerboseState();
    }
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

        // Compute per-module angle error (relative vs absolute encoder)
        double absAngle = state.getAbsoluteEncoderAngle();
        if (absAngle >= 0) {
          double absNormalized = SwerveUtils.normalizeDegrees(absAngle);
          double error = SwerveUtils.normalizeDegrees(state.getAngle() - absNormalized);
          telemetry.moduleAngleErrorDegrees[i] = error;
        }
      }
    }
    // post it!
    telemetry.post();
  }

  public SwerveTelemetry getSwerveTelemetry() {
    return telemetry;
  }
}
