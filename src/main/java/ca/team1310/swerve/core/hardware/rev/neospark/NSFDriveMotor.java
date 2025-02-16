package ca.team1310.swerve.core.hardware.rev.neospark;

import ca.team1310.swerve.core.config.MotorConfig;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;

/**
 * @author Tony Field
 * @since 2025-01-26 07:15
 */
public class NSFDriveMotor extends NSDriveMotor<SparkFlex> {

    /**
     * Construct a properly configured drive motor.
     * @param canId The CAN ID of the motor
     * @param cfg The configuration of the motor
     * @param wheelRadiusMetres The radius of the wheel in metres
     * @param robotPeriodMillis  The period of the robot in milliseconds
     */
    public NSFDriveMotor(int canId, MotorConfig cfg, double wheelRadiusMetres, int robotPeriodMillis) {
        super(new SparkFlex(canId, SparkLowLevel.MotorType.kBrushless), cfg, wheelRadiusMetres, robotPeriodMillis);
    }
}
