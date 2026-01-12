package ca.team1310.swerve.gyro;

import ca.team1310.swerve.RunnymedeSwerveDrive;
import ca.team1310.swerve.RunnymedeSwerveSubsystem;
import ca.team1310.swerve.core.config.CoreSwerveConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class GetWheelRadiusCommand extends Command {

  private final RunnymedeSwerveDrive swerve;
  private final GyroAwareSwerveDrive gyroDrive;
  private double gyroOffset;
  private final double TARGET_DEGREES = 360 * 20;
  private double[] encoderOffsets = new double[4];
  private int finishedCount = 0;

  public GetWheelRadiusCommand(RunnymedeSwerveSubsystem swerveSubsystem) {
    addRequirements(swerveSubsystem);
    swerve = swerveSubsystem.getRunnymedeSwerveDrive();
    if (swerve instanceof GyroAwareSwerveDrive) {
      gyroDrive = (GyroAwareSwerveDrive) swerve;
    } else {
      gyroDrive = null;
    }
  }

  @Override
  public void initialize() {
    swerve.driveRobotOriented(0, 0, 0);
    gyroOffset = gyroDrive.getYawRaw();
    encoderOffsets = gyroDrive.getRawDriveEncoders();
    SmartDashboard.putNumber("swerve/wheel-radius", 0);
    finishedCount = 0;
  }

  @Override
  public void execute() {
    double gyro = gyroDrive.getYawRaw() - gyroOffset;
    double error = gyro - TARGET_DEGREES;

    if (error < -1) {
      swerve.driveRobotOriented(0, 0, .2);
      System.out.print("+");
      finishedCount = 0;
    } else if (error > 1) {
      swerve.driveRobotOriented(0, 0, -.2);
      System.out.print("-");
      finishedCount = 0;
    } else {
      swerve.driveRobotOriented(0, 0, 0);
      System.out.print("*");
      finishedCount++;
    }
    System.out.println("Gyro: " + gyro);
  }

  @Override
  public boolean isFinished() {
    if (gyroDrive == null) return true;

    double gyro = gyroDrive.getYawRaw() - gyroOffset;
    double error = gyro - TARGET_DEGREES;

    if (Math.abs(error) < 1 && finishedCount > 5) return true;

    return false;
  }

  @Override
  public void end(boolean interrupted) {
    if (gyroDrive == null) {
      System.out.println("This command requires a GyroAwareSwerveDrive");
      return;
    }

    double[] rawEncoders = gyroDrive.getRawDriveEncoders();

    double[] encoderDelta = new double[4];

    encoderDelta[0] = Math.abs(rawEncoders[0] - encoderOffsets[0]);
    encoderDelta[1] = Math.abs(rawEncoders[1] - encoderOffsets[1]);
    encoderDelta[2] = Math.abs(rawEncoders[2] - encoderOffsets[2]);
    encoderDelta[3] = Math.abs(rawEncoders[3] - encoderOffsets[3]);

    double averageEncoderDelta =
        (encoderDelta[0] + encoderDelta[1] + encoderDelta[2] + encoderDelta[3]) / 4;

    System.out.println("average encoder delta: " + averageEncoderDelta);

    CoreSwerveConfig cfg = gyroDrive.getCoreCfg();
    double gearRatio = cfg.frontLeftModuleConfig().driveMotorConfig().gearRatio();
    double frameDiagonal = Math.hypot(cfg.trackWidthMetres(), cfg.wheelBaseMetres());

    double wheelRotations = averageEncoderDelta / gearRatio;
    double distance = (TARGET_DEGREES / 360) * Math.PI * frameDiagonal;

    System.out.println("wheel rotations: " + wheelRotations);
    System.out.println("distance: " + distance);

    double wheelCircumference = distance / wheelRotations;
    double wheelRadius = wheelCircumference / (2 * Math.PI);

    System.out.println("WheelRadius: " + wheelRadius);
    SmartDashboard.putNumber("swerve/wheel-radius", wheelRadius);
  }
}
