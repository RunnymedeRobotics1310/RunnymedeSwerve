package ca.team1310.swerve.core;

import ca.team1310.swerve.RunnymedeSwerveDrive;
import ca.team1310.swerve.SwerveTelemetry;
import ca.team1310.swerve.core.config.CoreSwerveConfig;
import ca.team1310.swerve.utils.FieldPose;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import java.util.Arrays;

/**
 * The Core swerve drive object. This implements most core features of a swerve drive
 */
public abstract class CoreSwerveDrive implements RunnymedeSwerveDrive {

    private final SwerveModule[] modules;

    private final SwerveMath math;

    private double desiredVx;
    private double desiredVy;
    private double desiredOmega;

    private final SwerveTelemetry telemetry;
    protected final boolean isSimulation;

    /**
     * Construct a new CoreSwerveDrive object with the specified configuration.
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

        this.math = new SwerveMath(
            cfg.wheelBaseMetres(),
            cfg.trackWidthMetres(),
            cfg.maxAttainableModuleSpeedMetresPerSecond(),
            cfg.maxAchievableRotationalVelocityRadiansPerSecond()
        );

        this.telemetry = cfg.telemetry();
        this.telemetry.maxModuleSpeedMPS = cfg.maxAttainableModuleSpeedMetresPerSecond();
        this.telemetry.maxTranslationSpeedMPS = cfg.maxAttainableTranslationSpeedMetresPerSecond();
        this.telemetry.maxRotationalVelocityRadPS = cfg.maxAchievableRotationalVelocityRadiansPerSecond();
        this.telemetry.trackWidthMetres = cfg.trackWidthMetres();
        this.telemetry.wheelBaseMetres = cfg.wheelBaseMetres();
        this.telemetry.wheelRadiusMetres = cfg.wheelRadiusMetres();
    }

    public final void drive(double x, double y, double w) {
        desiredVx = x;
        desiredVy = y;
        desiredOmega = w;
        updateModules();
    }

    /*
     * Set the module states based on the desired speed and angle.
     * Note that if discretize() is used, this will need to be called
     * on a very regular periodic basis - the period needs to match
     * the period used in the discretize() call.
     *
     * TODO: consider moving this to a separate thread, or else periodic?
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

    @Override
    public final void periodic() {
        long start = RobotController.getFPGATime();
        periodicInternal();
        long deltaMicros = RobotController.getFPGATime() - start;
        long deltaMillis = deltaMicros / 1000;
        if (deltaMillis > 15) {
            System.out.println("RunnymedeSwerve periodic exceeded 15ms: " + deltaMillis + "ms");
        }
    }

    protected void periodicInternal() {
        populateTelemetry();
    }

    /**
     * Get the pose of the modules on the field given a robot pose. Uses the robot pose and adds the location of each module.
     * @param robotPose the pose of the robot on the field (referring to the center of the robot).
     * @return Poses of front left, front right, back left, back right modules.
     */
    protected FieldPose[] getModulePoses(FieldPose robotPose) {
        return Arrays.stream(modules)
            .map(m -> {
                Transform2d tx = new Transform2d(m.getLocation(), m.getState().angle);
                return robotPose.plus(tx);
            })
            .toArray(Pose2d[]::new);
    }

    /**
     * Get the states of the swerve modules, including their speed and angle.
     * @return an array of swerve module states in the order front left, front right, back left, back right.
     */
    protected SwerveModuleState[] getModuleStates() {
        return Arrays.stream(modules).map(SwerveModule::getState).toArray(SwerveModuleState[]::new);
    }

    @Override
    public void setModuleState(String moduleName, ModuleState desiredState) {
        // This is for TEST MODE ONLY!!! Not for internal use or drive use.
        for (SwerveModule module : modules) {
            if (module.getName().equals(moduleName)) {
                module.setDesiredState(desiredState);
                break;
            }
        }
        this.desiredChassisSpeeds = kinematics.toChassisSpeeds(getModuleStates());
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
                ModuleState lockState = new ModuleState();
                lockState.set(0, /* TODO: module position --> as angle */0);
                module.setDesiredState(lockState);
            }
        }

        return !moving;
    }

    private void populateTelemetry() {
        if (telemetry.enabled) {
            ChassisSpeeds measuredChassisSpeeds = kinematics.toChassisSpeeds(getModuleStates());
            telemetry.measuredChassisSpeeds[0] = measuredChassisSpeeds.vxMetersPerSecond;
            telemetry.measuredChassisSpeeds[1] = measuredChassisSpeeds.vyMetersPerSecond;
            telemetry.measuredChassisSpeeds[2] = measuredChassisSpeeds.omegaRadiansPerSecond;

            telemetry.desiredChassisSpeeds[0] = this.desiredVx;
            telemetry.desiredChassisSpeeds[1] = this.desiredVy;
            telemetry.desiredChassisSpeeds[2] = this.desiredOmega;

            for (int i = 0; i < modules.length; i++) {
                modules[i].populateTelemetry(telemetry, i);
            }
        }
    }
}
