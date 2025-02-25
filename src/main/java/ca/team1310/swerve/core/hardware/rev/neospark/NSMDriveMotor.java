package ca.team1310.swerve.core.hardware.rev.neospark;

import ca.team1310.swerve.core.config.MotorConfig;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;

/**
 * @author Tony Field
 * @since 2025-01-26 07:18
 */
public class NSMDriveMotor extends NSDriveMotor<SparkMax> {

    /**
     * Construct a properly configured drive motor.
     * @param canId The CAN ID of the motor
     * @param cfg The configuration of the motor
     * @param wheelRadiusMetres The radius of the wheel in metres
     * @param robotPeriodMillis  The period of the robot in milliseconds
     */
    public NSMDriveMotor(int canId, MotorConfig cfg, double wheelRadiusMetres, double maxAttainableModuleSpeedMps, int robotPeriodMillis) {
        super(new SparkMax(canId, SparkLowLevel.MotorType.kBrushless), cfg, wheelRadiusMetres, maxAttainableModuleSpeedMps, robotPeriodMillis);
    }
}
