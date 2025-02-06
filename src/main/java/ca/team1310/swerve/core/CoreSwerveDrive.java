package ca.team1310.swerve.core;

import ca.team1310.swerve.RunnymedeSwerveDrive;
import ca.team1310.swerve.SwerveTelemetry;
import ca.team1310.swerve.core.config.CoreSwerveConfig;
import ca.team1310.swerve.odometry.FieldPose;
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

    private final SwerveTelemetry telemetry;
    protected final boolean isSimulation;

    //
    // Thread controls
    //
    // module update thread
    private final Notifier notifier = new Notifier(this::updateModules);
    private static final double UPDATE_MODULE_PERIOD = 0.005;

    // module state refresher thread
    private final Notifier updateModuleStates = new Notifier(this::updateModuleStates);
    private static final double UPDATE_MODULE_STATE_PERIOD = 0.005;
    private short moduleStateUpdateCount = 0;
    // cycles for updating module state. Maximum 10000
    private final short ODOMETRY_UPDATE_CYCLES = 4;
    private final short VISION_UPDATE_CYCLES = 4 * 10;
    private final short TELEMETRY_UPDATE_CYCLES = 4 * 100;

    /**
     * Construct a new CoreSwerveDrive object with the specified configuration.
     *
     * @param cfg the configuration of the swerve drive
     */
    protected CoreSwerveDrive(CoreSwerveConfig cfg) {
        // order matters in case we want to use AdvantageScope
        this.modules = new SwerveModule[4];
        this.isSimulation = RobotBase.isSimulation();
        if (isSimulation) {
            this.modules[0] = new SwerveModuleSimulation(cfg.frontLeftModuleConfig());
            this.modules[1] = new SwerveModuleSimulation(cfg.frontRightModuleConfig());
            this.modules[2] = new SwerveModuleSimulation(cfg.backLeftModuleConfig());
            this.modules[3] = new SwerveModuleSimulation(cfg.backRightModuleConfig());
        } else {
            this.modules[0] = new SwerveModuleImpl(
                cfg.frontLeftModuleConfig(),
                (int) (cfg.robotPeriodSeconds() * 1000)
            );
            this.modules[1] = new SwerveModuleImpl(
                cfg.frontRightModuleConfig(),
                (int) (cfg.robotPeriodSeconds() * 1000)
            );
            this.modules[2] = new SwerveModuleImpl(cfg.backLeftModuleConfig(), (int) (cfg.robotPeriodSeconds() * 1000));
            this.modules[3] = new SwerveModuleImpl(
                cfg.backRightModuleConfig(),
                (int) (cfg.robotPeriodSeconds() * 1000)
            );
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

        notifier.setName("RunnymedeSwerve moduleUpdater");
        notifier.startPeriodic(UPDATE_MODULE_PERIOD);

        updateModuleStates.setName("RunnymedeSwerve updateModuleStates");
        updateModuleStates.startPeriodic(UPDATE_MODULE_STATE_PERIOD);

        this.telemetry = cfg.telemetry();
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
        this.telemetry.moduleWheelLocations[0] = cfg.frontLeftModuleConfig().xPositionMetres();
        this.telemetry.moduleWheelLocations[1] = cfg.frontLeftModuleConfig().yPositionMetres();
        this.telemetry.moduleWheelLocations[2] = cfg.frontRightModuleConfig().xPositionMetres();
        this.telemetry.moduleWheelLocations[3] = cfg.frontRightModuleConfig().yPositionMetres();
        this.telemetry.moduleWheelLocations[4] = cfg.backLeftModuleConfig().xPositionMetres();
        this.telemetry.moduleWheelLocations[5] = cfg.backLeftModuleConfig().yPositionMetres();
        this.telemetry.moduleWheelLocations[6] = cfg.backRightModuleConfig().xPositionMetres();
        this.telemetry.moduleWheelLocations[7] = cfg.backRightModuleConfig().yPositionMetres();
    }

    public final void drive(double x, double y, double w) {
        desiredVx = x;
        desiredVy = y;
        desiredOmega = w;
    }

    /*
     * Set the module states based on the desired speed and angle.
     * Note that if discretize() is used, this will need to be called
     * on a very regular periodic basis - the period needs to match
     * the period used in the discretize() call.
     */
    private void updateModules() {
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

        // calculate desired states
        math.calculateModuleSetpoints(desiredVx, desiredVy, desiredOmega);

        // set the module states
        this.modules[0].setDesiredState(math.getFrontLeft());
        this.modules[1].setDesiredState(math.getFrontRight());
        this.modules[2].setDesiredState(math.getBackLeft());
        this.modules[3].setDesiredState(math.getBackRight());
    }

    private void updateModuleStates() {
        boolean odometry = moduleStateUpdateCount % ODOMETRY_UPDATE_CYCLES == 0;
        boolean vision = moduleStateUpdateCount % VISION_UPDATE_CYCLES == 0;
        boolean telemetry = moduleStateUpdateCount % TELEMETRY_UPDATE_CYCLES == 0;
        if (moduleStateUpdateCount > 10000) {
            moduleStateUpdateCount = 0;
        } else {
            moduleStateUpdateCount++;
        }
        this.modules[0].updateState(odometry, vision, telemetry);
        this.modules[1].updateState(odometry, vision, telemetry);
        this.modules[2].updateState(odometry, vision, telemetry);
        this.modules[3].updateState(odometry, vision, telemetry);
    }

    /**
     * Get the pose of the modules on the field given a robot pose. Uses the robot pose and adds the location of each module.
     *
     * @param robotPose the pose of the robot on the field (referring to the center of the robot).
     * @return Poses of front left, front right, back left, back right modules.
     */
    protected FieldPose[] getModulePoses(FieldPose robotPose) {
        var poses = new FieldPose[4];
        for (int i = 0; i < modules.length; i++) {
            double robotX = robotPose.getX();
            double robotY = robotPose.getY();
            double robotTheta = robotPose.getTheta();
            double moduleX = modules[i].getLocation().getX();
            double moduleY = modules[i].getLocation().getY();

            double c = moduleY / moduleX;
            double moduleTheta = (Math.atan2(moduleY, moduleX) * 180) / Math.PI;
            double fieldTheta = (moduleTheta + robotTheta);
            double robotRelativeCoordFieldRelativeThetaModuleX = c * Math.cos(fieldTheta);
            double robotRelativeCoordFieldRelativeThetaModuleY = c * Math.sin(fieldTheta);
            double fieldRelativeModuleX = robotX + robotRelativeCoordFieldRelativeThetaModuleX;
            double fieldRelativeModuleY = robotY + robotRelativeCoordFieldRelativeThetaModuleY;

            poses[i] = new FieldPose(fieldRelativeModuleX, fieldRelativeModuleY, fieldTheta);
        }
        return poses;
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
    public void setModuleState(String moduleName, double speed, double angle) {
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
    public final boolean lock() {
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
                lockState.set(0, 45);
                module.setDesiredState(lockState);
            }
        }

        return !moving;
    }

    public void updateTelemetry(SwerveTelemetry telemetry) {
        if (telemetry.enabled) {
            // todo: these need to move to odometrySwerveDrive
            //            ChassisSpeeds measuredChassisSpeeds = kinematics.toChassisSpeeds(getModuleStates());
            //            telemetry.measuredChassisSpeeds[0] = measuredChassisSpeeds.vxMetersPerSecond;
            //            telemetry.measuredChassisSpeeds[1] = measuredChassisSpeeds.vyMetersPerSecond;
            //            telemetry.measuredChassisSpeeds[2] = measuredChassisSpeeds.omegaRadiansPerSecond;

            telemetry.desiredChassisSpeeds[0] = this.desiredVx;
            telemetry.desiredChassisSpeeds[1] = this.desiredVy;
            telemetry.desiredChassisSpeeds[2] = this.desiredOmega;

            for (int i = 0; i < modules.length; i++) {
                ModuleState state = modules[i].getState();
                // desired states
                telemetry.moduleDesiredStates[i * 2] = state.getDesiredAngle();
                telemetry.moduleDesiredStates[i * 2 + 1] = state.getDesiredSpeed();

                // measured states
                telemetry.moduleMeasuredStates[i * 2] = state.getPosition();
                telemetry.moduleMeasuredStates[i * 2 + 1] = state.getSpeed();

                // position information
                telemetry.moduleAngleMotorPositionDegrees[i] = state.getAngle();
                telemetry.moduleDriveMotorPositionMetres[i] = state.getPosition();
                telemetry.driveMotorOutputPower[i] = state.getDriveOutputPower();

                // angle encoder
                telemetry.moduleAbsoluteEncoderPositionDegrees[i] = state.getAbsoluteEncoderAngle();
            }
        }
    }

    @Override
    public void resetOdometry(FieldPose pose) {}

    @Override
    public FieldPose getPose() {
        return new FieldPose();
    }

    @Override
    public void zeroGyro() {}

    @Override
    public double getRoll() {
        return 0;
    }

    @Override
    public double getPitch() {
        return 0;
    }

    @Override
    public double getYaw() {
        return 0;
    }
}
