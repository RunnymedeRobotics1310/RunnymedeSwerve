package ca.team1310.swervedrive.core;

import ca.team1310.swervedrive.SwerveTelemetry;
import ca.team1310.swervedrive.core.config.CoreSwerveConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotBase;

import java.util.Arrays;

public class CoreSwerveDrive {
    private final SwerveModule[]          modules;
    protected final SwerveDriveKinematics kinematics;
    private final double                  robotPeriodSeconds;
    private final double                  maxModuleMPS;
    private final double                  maxTranslationMPS;
    private final double                  maxOmegaRadPerSec;

    private ChassisSpeeds                 desiredChassisSpeeds;
    private final SwerveTelemetry         telemetry;

    public CoreSwerveDrive(CoreSwerveConfig cfg) {
        // order matters in case we want to use AdvantageScope
        this.modules = new SwerveModule[4];
        if (RobotBase.isSimulation()) {
            this.modules[0] = new SwerveModuleSimulation(cfg.frontLeftModuleConfig());
            this.modules[1] = new SwerveModuleSimulation(cfg.frontRightModuleConfig());
            this.modules[2] = new SwerveModuleSimulation(cfg.backLeftModuleConfig());
            this.modules[3] = new SwerveModuleSimulation(cfg.backRightModuleConfig());
        }
        else {
            this.modules[0] = new SwerveModuleImpl(cfg.frontLeftModuleConfig());
            this.modules[1] = new SwerveModuleImpl(cfg.frontRightModuleConfig());
            this.modules[2] = new SwerveModuleImpl(cfg.backLeftModuleConfig());
            this.modules[3] = new SwerveModuleImpl(cfg.backRightModuleConfig());
        }

        this.kinematics                           = new SwerveDriveKinematics(
            Arrays.stream(modules).map(SwerveModule::getLocation).toArray(Translation2d[]::new));

        this.robotPeriodSeconds                   = cfg.robotPeriodSeconds();
        this.maxModuleMPS                         = cfg.maxAttainableModuleSpeedMetresPerSecond();
        this.maxTranslationMPS                    = cfg.maxAttainableTranslationSpeedMetresPerSecond();
        this.maxOmegaRadPerSec                    = cfg.maxAchievableRotationalVelocityRadiansPerSecond();

        this.telemetry                            = cfg.telemetry();
        this.telemetry.maxModuleSpeedMPS          = cfg.maxAttainableModuleSpeedMetresPerSecond();
        this.telemetry.maxTranslationSpeedMPS     = cfg.maxAttainableTranslationSpeedMetresPerSecond();
        this.telemetry.maxRotationalVelocityRadPS = cfg.maxAchievableRotationalVelocityRadiansPerSecond();
        this.telemetry.trackWidthMetres           = cfg.trackWidthMetres();
        this.telemetry.wheelBaseMetres            = cfg.wheelBaseMetres();
        this.telemetry.wheelRadiusMetres          = cfg.wheelRadiusMetres();
    }

    protected final SwerveModulePosition[] getModulePositions() {
        return Arrays.stream(modules).map(SwerveModule::getPosition).toArray(SwerveModulePosition[]::new);
    }

    protected Pose2d[] getModulePoses(Pose2d robotPose) {
        return Arrays.stream(modules).map(m -> {
            Transform2d tx = new Transform2d(m.getLocation(), m.getState().angle);
            return robotPose.plus(tx);
        }).toArray(Pose2d[]::new);
    }

    protected SwerveModuleState[] getStates() {
        return Arrays.stream(modules).map(SwerveModule::getState).toArray(SwerveModuleState[]::new);
    }

    public void setModuleState(String moduleName, SwerveModuleState desiredState) {
        // This is for TEST MODE ONLY!!! Not for internal use or drive use.
        for (SwerveModule module : modules) {
            if (module.getName().equals(moduleName)) {
                module.setDesiredState(desiredState);
                break;
            }
        }
        this.desiredChassisSpeeds = kinematics.toChassisSpeeds(getStates());
        populateTelemetry();
    }


    public final void drive(ChassisSpeeds rawDesiredRobotOrientedVelocity) {
        this.desiredChassisSpeeds = rawDesiredRobotOrientedVelocity;
        updateModules();
        populateTelemetry();
    }

    /*
     * TODO: Important: This needs to be done on a regular basis. We could delegate it to
     * a calling subsystem, but it would be more robust to do this internally.
     * The same applies to updateOdometry(). Possibly telemetry too.
     *
     * Should we spin up a new thread for this?
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
        ChassisSpeeds       discretized      = ChassisSpeeds.discretize(desiredChassisSpeeds, robotPeriodSeconds);

        // calculate desired states
        Translation2d       centerOfRotation = new Translation2d();
        SwerveModuleState[] states           = kinematics.toSwerveModuleStates(discretized, centerOfRotation);

        // ensure that we aren't trying to drive any module faster than it's capable of driving
        SwerveDriveKinematics.desaturateWheelSpeeds(states, discretized, maxModuleMPS, maxTranslationMPS, maxOmegaRadPerSec);

        // set the module states
        for (int i = 0; i < modules.length; i++) {
            modules[i].setDesiredState(states[i]);
        }
    }

    public final boolean lock() {

        boolean moving = false;

        // safety code to prevent locking if robot is moving
        for (SwerveModule swerveModule : modules) {
            // do not lock if moving more than 4cm/s
            if (Math.abs(swerveModule.getState().speedMetersPerSecond) > 0.04) {
                moving = true;
            }
        }

        if (!moving) {
            desiredChassisSpeeds = new ChassisSpeeds(0, 0, 0);

            // set speed to 0 and angle wheels to center
            for (SwerveModule module : modules) {
                module.setDesiredState(new SwerveModuleState(0.0, module.getPosition().angle));
            }
        }

        populateTelemetry();

        return !moving;
    }

    private void populateTelemetry() {
        ChassisSpeeds measuredChassisSpeeds = kinematics.toChassisSpeeds(getStates());
        telemetry.measuredChassisSpeeds[0] = measuredChassisSpeeds.vxMetersPerSecond;
        telemetry.measuredChassisSpeeds[1] = measuredChassisSpeeds.vyMetersPerSecond;
        telemetry.measuredChassisSpeeds[2] = measuredChassisSpeeds.omegaRadiansPerSecond;

        telemetry.desiredChassisSpeeds[0]  = desiredChassisSpeeds.vxMetersPerSecond;
        telemetry.desiredChassisSpeeds[1]  = desiredChassisSpeeds.vyMetersPerSecond;
        telemetry.desiredChassisSpeeds[2]  = desiredChassisSpeeds.omegaRadiansPerSecond;

        for (int i = 0; i < modules.length; i++) {
            modules[i].populateTelemetry(telemetry, i);
        }
    }
}
