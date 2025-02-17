package ca.team1310.swerve.core;

import static ca.team1310.swerve.core.config.TelemetryLevel.*;

import ca.team1310.swerve.RunnymedeSwerveDrive;
import ca.team1310.swerve.SwerveTelemetry;
import ca.team1310.swerve.core.config.CoreSwerveConfig;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * The Core swerve drive object. This implements most core features of a swerve drive
 */
public class CoreSwerveDrive implements RunnymedeSwerveDrive {

    private final SwerveModule[] modules;
    private final ModuleState[] moduleStates;

    private final SwerveMath math;

    private double desiredVx;
    private double desiredVy;
    private double desiredOmega;
    private final double MINIMUM_OMEGA_VALUE_RAD_PER_SEC = Math.toRadians(1);

    /**
     * The telemetry object for the swerve drive
     */
    protected final SwerveTelemetry telemetry;
    /**
     * Whether the robot is running in simulation mode
     */
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
    private static final int TELEMETRY_UPDATE_PERIOD = 500; // milliseconds

    /**
     * Construct a new CoreSwerveDrive object with the specified configuration.
     *
     * @param cfg the configuration of the swerve drive
     */
    protected CoreSwerveDrive(CoreSwerveConfig cfg) {
        // order matters in case we want to use AdvantageScope
        double dt = cfg.robotPeriodSeconds();
        this.modules = new SwerveModule[4];
        this.isSimulation = RobotBase.isSimulation();
        if (isSimulation) {
            this.modules[0] = new SwerveModuleSimulation(cfg.frontLeftModuleConfig());
            this.modules[1] = new SwerveModuleSimulation(cfg.frontRightModuleConfig());
            this.modules[2] = new SwerveModuleSimulation(cfg.backLeftModuleConfig());
            this.modules[3] = new SwerveModuleSimulation(cfg.backRightModuleConfig());
        } else {
            this.modules[0] = new SwerveModuleImpl(cfg.frontLeftModuleConfig(), (int) (dt * 1000));
            this.modules[1] = new SwerveModuleImpl(cfg.frontRightModuleConfig(), (int) (dt * 1000));
            this.modules[2] = new SwerveModuleImpl(cfg.backLeftModuleConfig(), (int) (dt * 1000));
            this.modules[3] = new SwerveModuleImpl(cfg.backRightModuleConfig(), (int) (dt * 1000));
        }

        this.moduleStates = new ModuleState[4];
        this.moduleStates[0] = modules[0].getState();
        this.moduleStates[1] = modules[1].getState();
        this.moduleStates[2] = modules[2].getState();
        this.moduleStates[3] = modules[3].getState();

        this.math = new SwerveMath(
            cfg.wheelBaseMetres(),
            cfg.trackWidthMetres(),
            cfg.maxAttainableModuleSpeedMetresPerSecond(),
            cfg.maxAchievableRotationalVelocityRadiansPerSecond()
        );

        this.telemetry = new SwerveTelemetry(4);
        this.telemetry.level = cfg.telemetryLevel();
        this.telemetry.maxModuleSpeedMPS = cfg.maxAttainableModuleSpeedMetresPerSecond();
        this.telemetry.maxTranslationSpeedMPS = cfg.maxAttainableTranslationSpeedMetresPerSecond();
        this.telemetry.maxRotationalVelocityRadPS = cfg.maxAchievableRotationalVelocityRadiansPerSecond();
        this.telemetry.trackWidthMetres = cfg.trackWidthMetres();
        this.telemetry.wheelBaseMetres = cfg.wheelBaseMetres();
        this.telemetry.wheelRadiusMetres = cfg.wheelRadiusMetres();
        this.telemetry.moduleNames[0] = cfg.frontLeftModuleConfig().name();
        this.telemetry.moduleNames[1] = cfg.frontRightModuleConfig().name();
        this.telemetry.moduleNames[2] = cfg.backLeftModuleConfig().name();
        this.telemetry.moduleNames[3] = cfg.backRightModuleConfig().name();
        this.telemetry.moduleWheelLocations[0] = cfg.frontLeftModuleConfig().location().getX();
        this.telemetry.moduleWheelLocations[1] = cfg.frontLeftModuleConfig().location().getY();
        this.telemetry.moduleWheelLocations[2] = cfg.frontRightModuleConfig().location().getX();
        this.telemetry.moduleWheelLocations[3] = cfg.frontRightModuleConfig().location().getY();
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
            isSimulation ? READ_MODULES_PERIOD_MILLIS / 1000.0 : TELEMETRY_UPDATE_PERIOD / 1000.0
        );
    }

    public final synchronized void drive(double x, double y, double w) {
        desiredVx = x;
        desiredVy = y;
        desiredOmega = Math.abs(w) < MINIMUM_OMEGA_VALUE_RAD_PER_SEC ? 0 : w;
    }

    /*
     * Set the module states based on the desired speed and angle.
     * Note that if discretize() is used, this will need to be called
     * on a very regular periodic basis - the period needs to match
     * the period used in the discretize() call.
     */
    private synchronized void updateModules() {
        /*
         * Correct the robot's trajectory while it is rotating using ChassisSpeeds.discretize()
         *
         * From
         * https://www.chiefdelphi.com/t/looking-for-an-explanation-of-chassisspeeds-discretize/
         * btwn Tony Field (1310 Mentor) and Tyler Veness WPILib developer (controls and API design)
         *
         * Consider a hypothetical swerve robot translating in a straight line while rotating around
         * its center. The chassis velocities required to do that follow sinusoids (i.e., they
         * continuously vary). The robot code can only update the velocity commands at discrete
         * intervals, so the actual robot follows an arc away from the desired path.
         *
         * ChassisSpeeds.discretize() compensates for the discretization error by selecting constant
         * translational and rotational velocity commands that make the robot’s arc intersect the
         * desired path at the end of the timestep, where the desired path has decoupled translation
         * and rotation.
         *
         * Note that this only cancels out one cause of drift from the desired path. Another cause
         * is the swerve’s feedback controllers not keeping up with the commands.
         *
         * Just like with swerve module heading optimization, all swerve code should be using this.
         */
        // todo: implement discretize
        //        ChassisSpeeds speeds = ChassisSpeeds.discretize(desiredVx, desiredVy, desiredOmega, UPDATE_MODULE_PERIOD);
        //        desiredVx = speeds.vxMetersPerSecond;
        //        desiredVy = speeds.vyMetersPerSecond;
        //        desiredOmega = speeds.omegaRadiansPerSecond;

        // calculate desired states
        math.calculateAndStoreModuleVelocities(desiredVx, desiredVy, desiredOmega);

        // set the module states
        this.modules[0].setDesiredState(math.getFrontLeft());
        this.modules[1].setDesiredState(math.getFrontRight());
        this.modules[2].setDesiredState(math.getBackLeft());
        this.modules[3].setDesiredState(math.getBackRight());
    }

    private synchronized void readModuleStates() {
        boolean odometry = moduleStateUpdateCount % ODOMETRY_UPDATE_CYCLES == 0;
        boolean telemetry = moduleStateUpdateCount % TELEMETRY_UPDATE_CYCLES == 0;
        if (moduleStateUpdateCount > 10000) {
            moduleStateUpdateCount = 0;
        } else {
            moduleStateUpdateCount++;
        }
        this.modules[3].readState(odometry, telemetry);
        this.modules[2].readState(odometry, telemetry);
        this.modules[1].readState(odometry, telemetry);
        this.modules[0].readState(odometry, telemetry);

        if (telemetry) {
            updateTelemetry(this.telemetry);
        }
    }

    /**
     * Get the states of the swerve modules, including their speed and angle.
     *
     * @return an array of swerve module states in the order front left, front right, back left, back right.
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
     * <p>
     * Overriding methods should add their own data to telemetry and then
     * call super.updateTelemetry(telemetry) to ensure all data is collected and
     * reported consistently.
     * <p>
     * The frequency at which this is called depends on how frequently the
     * core swerve drive calls it. This is not currently configurable.
     *
     * @param telemetry the telemetry object to update
     */
    protected synchronized void updateTelemetry(SwerveTelemetry telemetry) {
        if (telemetry.level == INPUT || telemetry.level == CALCULATED || telemetry.level == VERBOSE) {
            telemetry.desiredChassisSpeeds[0] = this.desiredVx;
            telemetry.desiredChassisSpeeds[1] = this.desiredVy;
            telemetry.desiredChassisSpeeds[2] = Math.toDegrees(this.desiredOmega);
        }

        for (int i = 0; i < modules.length; i++) {
            ModuleState state = modules[i].getState();

            if (telemetry.level == INPUT || telemetry.level == CALCULATED || telemetry.level == VERBOSE) {
                // desired states
                telemetry.moduleDesiredStates[i * 2] = state.getDesiredAngle();
                telemetry.moduleDesiredStates[i * 2 + 1] = state.getDesiredSpeed();
            }

            if (telemetry.level == CALCULATED || telemetry.level == VERBOSE) {
                // measured states
                telemetry.moduleMeasuredStates[i * 2] = state.getPosition();
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
