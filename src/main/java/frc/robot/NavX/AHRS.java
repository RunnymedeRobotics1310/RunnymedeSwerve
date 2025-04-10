/*----------------------------------------------------------------------------*/
/* Copyright (c) Kauai Labs 2015. All Rights Reserved.                        */
/*                                                                            */
/* Created in support of Team 2465 (Kauaibots).  Go Purple Wave!              */
/*                                                                            */
/* Open Source Software - may be modified and shared by FRC teams. Any        */
/* modifications to this code must be accompanied by the \License.txt file    */
/* in the root directory of the project.                                      */
/*----------------------------------------------------------------------------*/

package frc.robot.NavX;

import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.networktables.NTSendable;
import edu.wpi.first.networktables.NTSendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.NavX.AHRSProtocol.*;
import frc.robot.NavX.IMUProtocol.*;

/**
 * The AHRS class provides an interface to AHRS capabilities of the KauaiLabs navX Robotics
 * Navigation Sensor via SPI, I2C and Serial (TTL UART and USB) communications interfaces on the
 * RoboRIO.
 *
 * <p>The AHRS class enables access to basic connectivity and state information, as well as key
 * 6-axis and 9-axis orientation information (yaw, pitch, roll, compass heading, fused (9-axis)
 * heading and magnetic disturbance detection.
 *
 * <p>Additionally, the ARHS class also provides access to extended information including linear
 * acceleration, motion detection, rotation detection and sensor temperature.
 *
 * <p>If used with the navX Aero, the AHRS class also provides access to altitude, barometric
 * pressure and pressure sensor temperature data
 *
 * @author Scott
 */
public class AHRS implements NTSendable, AutoCloseable {

  /**
   * Identifies one of the three sensing axes on the navX sensor board. Note that these axes are
   * board-relative ("Board Frame"), and are not necessarily the same as the logical axes of the
   * chassis on which the sensor is mounted.
   *
   * <p>For more information on sensor orientation, please see the navX sensor <a
   * href=http://navx-mxp.kauailabs.com/installation/orientation-2/>Orientation</a> page.
   */
  public enum BoardAxis {
    kBoardAxisX(0),
    kBoardAxisY(1),
    kBoardAxisZ(2);

    private int value;

    private BoardAxis(int value) {
      this.value = value;
    }

    public int getValue() {
      return this.value;
    }
  };

  /**
   * Indicates which sensor board axis is used as the "yaw" (gravity) axis.
   *
   * <p>This selection may be modified via the <a
   * href=http://navx-mxp.kauailabs.com/installation/omnimount/>Omnimount</a> feature.
   */
  public static class BoardYawAxis {
    public BoardAxis board_axis;
    public boolean up;
  }
  ;

  /**
   * For use with serial communications, the SerialDataType specifies the type of data to be
   * streamed from the sensor. Due to limitations in the streaming bandwidth on some serial
   * interfaces, only a subset of all available data can be streamed.
   *
   * <p>Note that if communicating over I2C/SPI, all available data can be retrieved, so the
   * SerialDataType need only be specified if using serial communications.
   */
  public enum SerialDataType {
    /** (default): 6 and 9-axis processed data */
    kProcessedData(0),
    /** unprocessed data from each individual sensor */
    kRawData(1);

    private int value;

    private SerialDataType(int value) {
      this.value = value;
    }

    public int getValue() {
      return this.value;
    }
  };

  static final byte NAVX_DEFAULT_UPDATE_RATE_HZ = 60;
  static final int YAW_HISTORY_LENGTH = 10;
  static final short DEFAULT_ACCEL_FSR_G = 2;
  static final short DEFAULT_GYRO_FSR_DPS = 2000;
  static final float QUATERNION_HISTORY_SECONDS = 5.0f;

  /* Processed Data */

  volatile float yaw;
  volatile float pitch;
  volatile float roll;
  volatile float compass_heading;
  volatile float world_linear_accel_x;
  volatile float world_linear_accel_y;
  volatile float world_linear_accel_z;
  volatile float mpu_temp_c;
  volatile float fused_heading;
  volatile float altitude;
  volatile float baro_pressure;
  volatile boolean is_moving;
  volatile boolean is_rotating;
  volatile float baro_sensor_temp_c;
  volatile boolean altitude_valid;
  volatile boolean is_magnetometer_calibrated;
  volatile boolean magnetic_disturbance;
  volatile float quaternionW;
  volatile float quaternionX;
  volatile float quaternionY;
  volatile float quaternionZ;

  /* Integrated Data */
  float velocity[] = new float[3];
  float displacement[] = new float[3];

  /* Raw Data */
  volatile short raw_gyro_x;
  volatile short raw_gyro_y;
  volatile short raw_gyro_z;
  volatile short raw_accel_x;
  volatile short raw_accel_y;
  volatile short raw_accel_z;
  volatile short cal_mag_x;
  volatile short cal_mag_y;
  volatile short cal_mag_z;

  /* Configuration/Status */
  volatile byte update_rate_hz;
  volatile short accel_fsr_g = DEFAULT_ACCEL_FSR_G;
  volatile short gyro_fsr_dps = DEFAULT_GYRO_FSR_DPS;
  volatile short capability_flags;
  volatile byte op_status;
  volatile short sensor_status;
  volatile byte cal_status;
  volatile byte selftest_status;

  /* Board ID */
  volatile byte board_type;
  volatile byte hw_rev;
  volatile byte fw_ver_major;
  volatile byte fw_ver_minor;

  long last_sensor_timestamp;
  double last_update_time;

  InertialDataIntegrator integrator;
  ContinuousAngleTracker yaw_angle_tracker;
  OffsetTracker yaw_offset_tracker;
  IIOProvider io;

  BoardCapabilities board_capabilities;
  IOCompleteNotification io_complete_sink;
  IOThread io_thread;

  private ITimestampedDataSubscriber callbacks[];
  private Object callback_contexts[];
  private final int MAX_NUM_CALLBACKS = 3;

  private boolean enable_boardlevel_yawreset;
  private double last_yawreset_request_timestamp;
  private double last_yawreset_while_calibrating_request_timestamp;
  private int successive_suppressed_yawreset_request_count;
  private boolean disconnect_startupcalibration_recovery_pending;
  private boolean logging_enabled;

  /* SimDevice Variables */
  private SimDevice m_simDevice;

  /***********************************************************/
  /* Public Interface Implementation                         */
  /***********************************************************/

  /**
   * Constructs the AHRS class using SPI communication, overriding the default update rate with a
   * custom rate which may be from 4 to 200, representing the number of updates per second sent by
   * the sensor.
   *
   * <p>This constructor should be used if communicating via SPI.
   *
   * <p>Note that increasing the update rate may increase the CPU utilization.
   *
   * <p>
   *
   * @param spi_port_id SPI Port to use
   * @param update_rate_hz Custom Update Rate (Hz)
   */
  public AHRS(SPI.Port spi_port_id, byte update_rate_hz) {
    commonInit(update_rate_hz);
    io =
        new RegisterIO(
            new RegisterIO_SPI(new SPI(spi_port_id)),
            update_rate_hz,
            io_complete_sink,
            board_capabilities);
    SendableRegistry.addLW(this, "navX-Sensor", spi_port_id.value);
    io_thread.start();
  }

  /**
   * The AHRS class provides an interface to AHRS capabilities of the KauaiLabs navX Robotics
   * Navigation Sensor via SPI, I2C and Serial (TTL UART and USB) communications interfaces on the
   * RoboRIO.
   *
   * <p>The AHRS class enables access to basic connectivity and state information, as well as key
   * 6-axis and 9-axis orientation information (yaw, pitch, roll, compass heading, fused (9-axis)
   * heading and magnetic disturbance detection.
   *
   * <p>Additionally, the ARHS class also provides access to extended information including linear
   * acceleration, motion detection, rotation detection and sensor temperature.
   *
   * <p>If used with the navX Aero, the AHRS class also provides access to altitude, barometric
   * pressure and pressure sensor temperature data
   *
   * <p>This constructor allows the specification of a custom SPI bitrate, in bits/second.
   *
   * @param spi_port_id SPI Port to use
   * @param spi_bitrate SPI bitrate (Maximum: 2,000,000)
   * @param update_rate_hz Custom Update Rate (Hz)
   */
  public AHRS(SPI.Port spi_port_id, int spi_bitrate, byte update_rate_hz) {
    Tracer.Trace("Instantiating navX-Sensor on SPI Port %s.\n", spi_port_id.toString());
    commonInit(update_rate_hz);
    io =
        new RegisterIO(
            new RegisterIO_SPI(new SPI(spi_port_id), spi_bitrate),
            update_rate_hz,
            io_complete_sink,
            board_capabilities);
    SendableRegistry.addLW(this, "navX-Sensor", spi_port_id.value);
    io_thread.start();
  }

  /**
   * Constructs the AHRS class using SPI communication and the default update rate.
   *
   * <p>This constructor should be used if communicating via SPI.
   */
  public AHRS() {
    this(SPI.Port.kMXP);
  }

  /**
   * Constructs the AHRS class using SPI communication and the default update rate.
   *
   * <p>This constructor should be used if communicating via SPI.
   *
   * <p>
   *
   * @param spi_port_id SPI port to use.
   */
  public AHRS(SPI.Port spi_port_id) {
    this(spi_port_id, NAVX_DEFAULT_UPDATE_RATE_HZ);
  }

  /**
   * Returns the current pitch value (in degrees, from -180 to 180) reported by the sensor. Pitch is
   * a measure of rotation around the X Axis.
   *
   * @return The current pitch value in degrees (-180 to 180).
   */
  public float getPitch() {
    return pitch;
  }

  /**
   * Returns the current roll value (in degrees, from -180 to 180) reported by the sensor. Roll is a
   * measure of rotation around the X Axis.
   *
   * @return The current roll value in degrees (-180 to 180).
   */
  public float getRoll() {
    return roll;
  }

  /**
   * Returns the current yaw value (in degrees, from -180 to 180) reported by the sensor. Yaw is a
   * measure of rotation around the Z Axis (which is perpendicular to the earth).
   *
   * <p>Note that the returned yaw value will be offset by a user-specified offset value; this
   * user-specified offset value is set by invoking the zeroYaw() method.
   *
   * @return The current yaw value in degrees (-180 to 180).
   */
  public float getYaw() {
    if (enable_boardlevel_yawreset && board_capabilities.isBoardYawResetSupported()) {
      return this.yaw;
    } else {
      return (float) yaw_offset_tracker.applyOffset(this.yaw);
    }
  }

  /**
   * Returns the current tilt-compensated compass heading value (in degrees, from 0 to 360) reported
   * by the sensor.
   *
   * <p>Note that this value is sensed by a magnetometer, which can be affected by nearby magnetic
   * fields (e.g., the magnetic fields generated by nearby motors).
   *
   * <p>Before using this value, ensure that (a) the magnetometer has been calibrated and (b) that a
   * magnetic disturbance is not taking place at the instant when the compass heading was generated.
   *
   * @return The current tilt-compensated compass heading, in degrees (0-360).
   */
  public float getCompassHeading() {
    return compass_heading;
  }

  static final int NUM_SUPPRESSED_SUCCESSIVE_YAWRESET_MESSAGES = 5;
  static final double SUPPRESSED_SUCESSIVE_YAWRESET_PERIOD_SECONDS = 0.2;

  /**
   * Sets the user-specified yaw offset to the current yaw value reported by the sensor.
   *
   * <p>This user-specified yaw offset is automatically subtracted from subsequent yaw values
   * reported by the getYaw() method.
   *
   * <p>NOTE: This method has no effect if the sensor is currently calibrating, since resetting the
   * yaw will interfere with the calibration process.
   */
  public void zeroYaw() {
    double curr_timestamp = Timer.getFPGATimestamp();
    double delta_time_since_last_yawreset_request =
        curr_timestamp - last_yawreset_request_timestamp;
    if (delta_time_since_last_yawreset_request < SUPPRESSED_SUCESSIVE_YAWRESET_PERIOD_SECONDS) {
      successive_suppressed_yawreset_request_count++;
      if ((successive_suppressed_yawreset_request_count
              % NUM_SUPPRESSED_SUCCESSIVE_YAWRESET_MESSAGES)
          == 1) {
        if (logging_enabled)
          Tracer.Trace(
              "navX-Sensor rapidly-repeated Yaw Reset ignored%s\n",
              ((successive_suppressed_yawreset_request_count
                      < NUM_SUPPRESSED_SUCCESSIVE_YAWRESET_MESSAGES)
                  ? "."
                  : (" (repeated messages suppressed).")));
      }
      return;
    }

    if (isCalibrating()) {
      double delta_time_since_last_yawreset_while_calibrating_request =
          curr_timestamp - last_yawreset_while_calibrating_request_timestamp;
      if ((delta_time_since_last_yawreset_while_calibrating_request
          > SUPPRESSED_SUCESSIVE_YAWRESET_PERIOD_SECONDS)) {
        Tracer.Trace(
            "navX-Sensor Yaw Reset request ignored - startup calibration is currently in progress.\n");
      }
      last_yawreset_while_calibrating_request_timestamp = curr_timestamp;
      return;
    }

    successive_suppressed_yawreset_request_count = 0;

    last_yawreset_request_timestamp = curr_timestamp;
    if (enable_boardlevel_yawreset && board_capabilities.isBoardYawResetSupported()) {
      io.zeroYaw();
      Tracer.Trace("navX-Sensor Board-level Yaw Reset requested.\n");
      /* Note:  Notification is deferred until action is complete. */
    } else {
      yaw_offset_tracker.setOffset();
      /* Notification occurs immediately. */
      io_complete_sink.yawResetComplete();
    }
  }

  /**
   * Returns true if the sensor is currently performing automatic gyro/accelerometer calibration.
   * Automatic calibration occurs when the sensor is initially powered on, during which time the
   * sensor should be held still, with the Z-axis pointing up (perpendicular to the earth).
   *
   * <p>NOTE: During this automatic calibration, the yaw, pitch and roll values returned may not be
   * accurate.
   *
   * <p>Once calibration is complete, the sensor will automatically remove an internal yaw offset
   * value from all reported values.
   *
   * <p>
   *
   * @return Returns true if the sensor is currently automatically calibrating the gyro and
   *     accelerometer sensors.
   */
  public boolean isCalibrating() {
    return !((cal_status & AHRSProtocol.NAVX_CAL_STATUS_IMU_CAL_STATE_MASK)
        == AHRSProtocol.NAVX_CAL_STATUS_IMU_CAL_COMPLETE);
  }

  /**
   * Indicates whether the sensor is currently connected to the host computer. A connection is
   * considered established whenever communication with the sensor has occurred recently.
   *
   * <p>
   *
   * @return Returns true if a valid update has been recently received from the sensor.
   */
  public boolean isConnected() {
    return io.isConnected();
  }

  /**
   * Returns the count in bytes of data received from the sensor. This could can be useful for
   * diagnosing connectivity issues.
   *
   * <p>If the byte count is increasing, but the update count (see getUpdateCount()) is not, this
   * indicates a software misconfiguration.
   *
   * @return The number of bytes received from the sensor.
   */
  public double getByteCount() {
    return io.getByteCount();
  }

  /**
   * Returns the navX-Model device's currently configured update rate. Note that the update rate
   * that can actually be realized is a value evenly divisible by the navX-Model device's internal
   * motion processor sample clock (200Hz). Therefore, the rate that is returned may be lower than
   * the requested sample rate.
   *
   * <p>The actual sample rate is rounded down to the nearest integer that is divisible by the
   * number of Digital Motion Processor clock ticks. For instance, a request for 58 Hertz will
   * result in an actual rate of 66Hz (200 / (200 / 58), using integer math.
   *
   * @return Returns the current actual update rate in Hz (cycles per second).
   */
  public int getActualUpdateRate() {
    byte actual_update_rate = getActualUpdateRateInternal((byte) getRequestedUpdateRate());
    return (int) (actual_update_rate & 0xFF);
  }

  private byte getActualUpdateRateInternal(byte update_rate) {
    final int NAVX_MOTION_PROCESSOR_UPDATE_RATE_HZ = 200;
    int integer_update_rate = (int) (update_rate & 0xFF);
    int realized_update_rate =
        NAVX_MOTION_PROCESSOR_UPDATE_RATE_HZ
            / (NAVX_MOTION_PROCESSOR_UPDATE_RATE_HZ / integer_update_rate);
    return (byte) realized_update_rate;
  }

  /**
   * Returns the currently requested update rate. rate. Note that not every update rate can actually
   * be realized, since the actual update rate must be a value evenly divisible by the navX-Model
   * device's internal motion processor sample clock (200Hz).
   *
   * <p>To determine the actual update rate, use the {@link #getActualUpdateRate()} method.
   *
   * @return Returns the requested update rate in Hz (cycles per second).
   */
  public int getRequestedUpdateRate() {
    return (int) (this.update_rate_hz & 0xFF);
  }

  /**
   * Returns the count of valid updates which have been received from the sensor. This count should
   * increase at the same rate indicated by the configured update rate.
   *
   * @return The number of valid updates received from the sensor.
   */
  public double getUpdateCount() {
    return io.getUpdateCount();
  }

  /**
   * Returns the sensor timestamp corresponding to the last sample retrieved from the sensor. Note
   * that this sensor timestamp is only provided when the Register-based IO methods (SPI, I2C) are
   * used; sensor timestamps are not provided when Serial-based IO methods (TTL UART, USB) are used.
   *
   * @return The sensor timestamp corresponding to the current AHRS sensor data.
   */
  public long getLastSensorTimestamp() {
    return this.last_sensor_timestamp;
  }

  /**
   * Returns the current linear acceleration in the X-axis (in G).
   *
   * <p>World linear acceleration refers to raw acceleration data, which has had the gravity
   * component removed, and which has been rotated to the same reference frame as the current yaw
   * value. The resulting value represents the current acceleration in the x-axis of the body (e.g.,
   * the robot) on which the sensor is mounted.
   *
   * <p>
   *
   * @return Current world linear acceleration in the X-axis (in G).
   */
  public float getWorldLinearAccelX() {
    return this.world_linear_accel_x;
  }

  /**
   * Returns the current linear acceleration in the Y-axis (in G).
   *
   * <p>World linear acceleration refers to raw acceleration data, which has had the gravity
   * component removed, and which has been rotated to the same reference frame as the current yaw
   * value. The resulting value represents the current acceleration in the Y-axis of the body (e.g.,
   * the robot) on which the sensor is mounted.
   *
   * <p>
   *
   * @return Current world linear acceleration in the Y-axis (in G).
   */
  public float getWorldLinearAccelY() {
    return this.world_linear_accel_y;
  }

  /**
   * Returns the current linear acceleration in the Z-axis (in G).
   *
   * <p>World linear acceleration refers to raw acceleration data, which has had the gravity
   * component removed, and which has been rotated to the same reference frame as the current yaw
   * value. The resulting value represents the current acceleration in the Z-axis of the body (e.g.,
   * the robot) on which the sensor is mounted.
   *
   * <p>
   *
   * @return Current world linear acceleration in the Z-axis (in G).
   */
  public float getWorldLinearAccelZ() {
    return this.world_linear_accel_z;
  }

  /**
   * Indicates if the sensor is currently detecting motion, based upon the X and Y-axis world linear
   * acceleration values. If the sum of the absolute values of the X and Y axis exceed a "motion
   * threshold", the motion state is indicated.
   *
   * <p>
   *
   * @return Returns true if the sensor is currently detecting motion.
   */
  public boolean isMoving() {
    return is_moving;
  }

  /**
   * Indicates if the sensor is currently detecting yaw rotation, based upon whether the change in
   * yaw over the last second exceeds the "Rotation Threshold."
   *
   * <p>Yaw Rotation can occur either when the sensor is rotating, or when the sensor is not
   * rotating AND the current gyro calibration is insufficiently calibrated to yield the standard
   * yaw drift rate.
   *
   * <p>
   *
   * @return Returns true if the sensor is currently detecting motion.
   */
  public boolean isRotating() {
    return is_rotating;
  }

  /**
   * Returns the current barometric pressure, based upon calibrated readings from the onboard
   * pressure sensor. This value is in units of millibar.
   *
   * <p>NOTE: This value is only valid for a navX Aero. To determine whether this value is valid,
   * see isAltitudeValid().
   *
   * @return Returns current barometric pressure (navX Aero only).
   */
  public float getBarometricPressure() {
    return baro_pressure;
  }

  /**
   * Returns the current altitude, based upon calibrated readings from a barometric pressure sensor,
   * and the currently-configured sea-level barometric pressure [navX Aero only]. This value is in
   * units of meters.
   *
   * <p>NOTE: This value is only valid sensors including a pressure sensor. To determine whether
   * this value is valid, see isAltitudeValid().
   *
   * <p>
   *
   * @return Returns current altitude in meters (as long as the sensor includes an installed
   *     on-board pressure sensor).
   */
  public float getAltitude() {
    return altitude;
  }

  /**
   * Indicates whether the current altitude (and barometric pressure) data is valid. This value will
   * only be true for a sensor with an onboard pressure sensor installed.
   *
   * <p>If this value is false for a board with an installed pressure sensor, this indicates a
   * malfunction of the onboard pressure sensor.
   *
   * <p>
   *
   * @return Returns true if a working pressure sensor is installed.
   */
  public boolean isAltitudeValid() {
    return this.altitude_valid;
  }

  /**
   * Returns the "fused" (9-axis) heading.
   *
   * <p>The 9-axis heading is the fusion of the yaw angle, the tilt-corrected compass heading, and
   * magnetic disturbance detection. Note that the magnetometer calibration procedure is required in
   * order to achieve valid 9-axis headings.
   *
   * <p>The 9-axis Heading represents the sensor's best estimate of current heading, based upon the
   * last known valid Compass Angle, and updated by the change in the Yaw Angle since the last known
   * valid Compass Angle. The last known valid Compass Angle is updated whenever a Calibrated
   * Compass Angle is read and the sensor has recently rotated less than the Compass Noise Bandwidth
   * (~2 degrees).
   *
   * @return Fused Heading in Degrees (range 0-360)
   */
  public float getFusedHeading() {
    return fused_heading;
  }

  /**
   * Indicates whether the current magnetic field strength diverges from the calibrated value for
   * the earth's magnetic field by more than the currently- configured Magnetic Disturbance Ratio.
   *
   * <p>This function will always return false if the sensor's magnetometer has not yet been
   * calibrated; see isMagnetometerCalibrated().
   *
   * @return true if a magnetic disturbance is detected (or the magnetometer is uncalibrated).
   */
  public boolean isMagneticDisturbance() {
    return magnetic_disturbance;
  }

  /**
   * Indicates whether the magnetometer has been calibrated.
   *
   * <p>Magnetometer Calibration must be performed by the user.
   *
   * <p>Note that if this function does indicate the magnetometer is calibrated, this does not
   * necessarily mean that the calibration quality is sufficient to yield valid compass headings.
   *
   * <p>
   *
   * @return Returns true if magnetometer calibration has been performed.
   */
  public boolean isMagnetometerCalibrated() {
    return is_magnetometer_calibrated;
  }

  /* Unit Quaternions */

  /**
   * Returns the imaginary portion (W) of the Orientation Quaternion which fully describes the
   * current sensor orientation with respect to the reference angle defined as the angle at which
   * the yaw was last "zeroed".
   *
   * <p>Each quaternion value (W,X,Y,Z) is expressed as a value ranging from -2 to 2. This total
   * range (4) can be associated with a unit circle, since each circle is comprised of 4 PI Radians.
   *
   * <p>For more information on Quaternions and their use, please see this <a
   * href=https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation>definition</a>.
   *
   * @return Returns the imaginary portion (W) of the quaternion.
   */
  public float getQuaternionW() {
    return quaternionW;
  }

  /**
   * Returns the real portion (X axis) of the Orientation Quaternion which fully describes the
   * current sensor orientation with respect to the reference angle defined as the angle at which
   * the yaw was last "zeroed".
   *
   * <p>Each quaternion value (W,X,Y,Z) is expressed as a value ranging from -2 to 2. This total
   * range (4) can be associated with a unit circle, since each circle is comprised of 4 PI Radians.
   *
   * <p>For more information on Quaternions and their use, please see this <a
   * href=https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation>description</a>.
   *
   * @return Returns the real portion (X) of the quaternion.
   */
  public float getQuaternionX() {
    return quaternionX;
  }

  /**
   * Returns the real portion (X axis) of the Orientation Quaternion which fully describes the
   * current sensor orientation with respect to the reference angle defined as the angle at which
   * the yaw was last "zeroed".
   *
   * <p>Each quaternion value (W,X,Y,Z) is expressed as a value ranging from -2 to 2. This total
   * range (4) can be associated with a unit circle, since each circle is comprised of 4 PI Radians.
   *
   * <p>For more information on Quaternions and their use, please see:
   *
   * <p>https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation
   *
   * @return Returns the real portion (X) of the quaternion.
   */
  public float getQuaternionY() {
    return quaternionY;
  }

  /**
   * Returns the real portion (X axis) of the Orientation Quaternion which fully describes the
   * current sensor orientation with respect to the reference angle defined as the angle at which
   * the yaw was last "zeroed".
   *
   * <p>Each quaternion value (W,X,Y,Z) is expressed as a value ranging from -2 to 2. This total
   * range (4) can be associated with a unit circle, since each circle is comprised of 4 PI Radians.
   *
   * <p>For more information on Quaternions and their use, please see:
   *
   * <p>https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation
   *
   * @return Returns the real portion (X) of the quaternion.
   */
  public float getQuaternionZ() {
    return quaternionZ;
  }

  /**
   * Zeros the displacement integration variables. Invoke this at the moment when integration
   * begins.
   */
  public void resetDisplacement() {
    if (board_capabilities.isDisplacementSupported()) {
      io.zeroDisplacement();
    } else {
      integrator.resetDisplacement();
    }
  }

  /**
   * Each time new linear acceleration samples are received, this function should be invoked. This
   * function transforms acceleration in G to meters/sec^2, then converts this value to Velocity in
   * meters/sec (based upon velocity in the previous sample). Finally, this value is converted to
   * displacement in meters, and integrated.
   *
   * @return none.
   */
  private void updateDisplacement(
      float accel_x_g, float accel_y_g, int update_rate_hz, boolean is_moving) {
    integrator.updateDisplacement(accel_x_g, accel_y_g, update_rate_hz, is_moving);
  }

  /**
   * Returns the velocity (in meters/sec) of the X axis [Experimental].
   *
   * <p>NOTE: This feature is experimental. Velocity measures rely on integration of acceleration
   * values from MEMS accelerometers which yield "noisy" values. The resulting velocities are not
   * known to be very accurate.
   *
   * @return Current Velocity (in meters/squared).
   */
  public float getVelocityX() {
    return (board_capabilities.isDisplacementSupported() ? velocity[0] : integrator.getVelocityX());
  }

  /**
   * Returns the velocity (in meters/sec) of the Y axis [Experimental].
   *
   * <p>NOTE: This feature is experimental. Velocity measures rely on integration of acceleration
   * values from MEMS accelerometers which yield "noisy" values. The resulting velocities are not
   * known to be very accurate.
   *
   * @return Current Velocity (in meters/squared).
   */
  public float getVelocityY() {
    return (board_capabilities.isDisplacementSupported() ? velocity[1] : integrator.getVelocityY());
  }

  /**
   * Returns the velocity (in meters/sec) of the Z axis [Experimental].
   *
   * <p>NOTE: This feature is experimental. Velocity measures rely on integration of acceleration
   * values from MEMS accelerometers which yield "noisy" values. The resulting velocities are not
   * known to be very accurate.
   *
   * @return Current Velocity (in meters/squared).
   */
  public float getVelocityZ() {
    return (board_capabilities.isDisplacementSupported() ? velocity[2] : 0.f);
  }

  /**
   * Returns the displacement (in meters) of the X axis since resetDisplacement() was last invoked
   * [Experimental].
   *
   * <p>NOTE: This feature is experimental. Displacement measures rely on double-integration of
   * acceleration values from MEMS accelerometers which yield "noisy" values. The resulting
   * displacement are not known to be very accurate, and the amount of error increases quickly as
   * time progresses.
   *
   * @return Displacement since last reset (in meters).
   */
  public float getDisplacementX() {
    return (board_capabilities.isDisplacementSupported()
        ? displacement[0]
        : integrator.getDisplacementX());
  }

  /**
   * Returns the displacement (in meters) of the Y axis since resetDisplacement() was last invoked
   * [Experimental].
   *
   * <p>NOTE: This feature is experimental. Displacement measures rely on double-integration of
   * acceleration values from MEMS accelerometers which yield "noisy" values. The resulting
   * displacement are not known to be very accurate, and the amount of error increases quickly as
   * time progresses.
   *
   * @return Displacement since last reset (in meters).
   */
  public float getDisplacementY() {
    return (board_capabilities.isDisplacementSupported()
        ? displacement[1]
        : integrator.getDisplacementY());
  }

  /**
   * Returns the displacement (in meters) of the Z axis since resetDisplacement() was last invoked
   * [Experimental].
   *
   * <p>NOTE: This feature is experimental. Displacement measures rely on double-integration of
   * acceleration values from MEMS accelerometers which yield "noisy" values. The resulting
   * displacement are not known to be very accurate, and the amount of error increases quickly as
   * time progresses.
   *
   * @return Displacement since last reset (in meters).
   */
  public float getDisplacementZ() {
    return (board_capabilities.isDisplacementSupported() ? displacement[2] : 0.f);
  }

  /**
   * Registers a callback interface. This interface will be called back when new data is available,
   * based upon a change in the sensor timestamp.
   *
   * <p>Note that this callback will occur within the context of the device IO thread, which is not
   * the same thread context the caller typically executes in.
   *
   * @param callback The callback object to be invoked when callbacks occur
   * @param callback_context The callback context object to be passed as a parameter to the callback
   *     object.
   * @return returns true if callback was successfully registered.
   */
  public boolean registerCallback(ITimestampedDataSubscriber callback, Object callback_context) {
    boolean registered = false;
    for (int i = 0; i < this.callbacks.length; i++) {
      if (this.callbacks[i] == null) {
        this.callbacks[i] = callback;
        this.callback_contexts[i] = callback_context;
        registered = true;
        break;
      }
    }
    return registered;
  }

  /**
   * Deregisters a previously registered callback interface.
   *
   * <p>Be sure to deregister any callback which have been previously registered, to ensure that the
   * object implementing the callback interface does not continue to be accessed when no longer
   * necessary.
   *
   * @param callback The previously-registered callback object to be deregistered.
   * @return returns true if callback was successfully deregistered.
   */
  public boolean deregisterCallback(ITimestampedDataSubscriber callback) {
    boolean deregistered = false;
    for (int i = 0; i < this.callbacks.length; i++) {
      if (this.callbacks[i] == callback) {
        this.callbacks[i] = null;
        deregistered = true;
        break;
      }
    }
    return deregistered;
  }

  /***********************************************************/
  /* Internal Implementation                                  */
  /***********************************************************/

  private void commonInit(byte update_rate_hz) {
    Tracer.Trace("navX-Sensor Java library for FRC\n");
    HAL.report(tResourceType.kResourceType_NavX, 0);
    this.board_capabilities = new BoardCapabilities();
    this.io_complete_sink = new IOCompleteNotification();
    this.io_thread = new IOThread();
    this.update_rate_hz = update_rate_hz;
    integrator = new InertialDataIntegrator();
    yaw_offset_tracker = new OffsetTracker(YAW_HISTORY_LENGTH);
    yaw_angle_tracker = new ContinuousAngleTracker();
    this.callbacks = new ITimestampedDataSubscriber[MAX_NUM_CALLBACKS];
    this.callback_contexts = new Object[MAX_NUM_CALLBACKS];
    this.enable_boardlevel_yawreset = false;
    this.last_yawreset_request_timestamp = 0;
    this.last_yawreset_while_calibrating_request_timestamp = 0;
    this.successive_suppressed_yawreset_request_count = 0;
    this.disconnect_startupcalibration_recovery_pending = false;
    this.logging_enabled = false;

    // Construct SimDevice (only succeeds in simulation environments)
    m_simDevice = SimDevice.create("navX-Sensor", 0);
  }

  /**
   * Returns the total accumulated yaw angle (Z Axis, in degrees) reported by the sensor.
   *
   * <p>NOTE: The angle is continuous, meaning it's range is beyond 360 degrees. This ensures that
   * algorithms that wouldn't want to see a discontinuity in the gyro output as it sweeps past 0 on
   * the second time around.
   *
   * <p>Note that the returned yaw value will be offset by a user-specified offset value; this
   * user-specified offset value is set by invoking the zeroYaw() method.
   *
   * <p>
   *
   * @return The current total accumulated yaw angle (Z axis) of the robot in degrees. This heading
   *     is based on integration of the returned rate from the Z-axis (yaw) gyro.
   */
  public double getAngle() {
    return yaw_angle_tracker.getAngle();
  }

  /**
   * Return the rate of rotation of the yaw (Z-axis) gyro, in degrees per second.
   *
   * <p>The rate is based on the most recent reading of the yaw gyro angle.
   *
   * <p>
   *
   * @return The current rate of change in yaw angle (in degrees per second)
   */
  public double getRate() {
    return yaw_angle_tracker.getRate();
  }

  /**
   * Sets an amount of angle to be automatically added before returning a angle from the getAngle()
   * method. This allows users of the getAngle() method to logically rotate the sensor by a given
   * amount of degrees.
   *
   * <p>NOTE 1: The adjustment angle is <b>only</b> applied to the value returned from getAngle() -
   * it does not adjust the value returned from getYaw(), nor any of the quaternion values.
   *
   * <p>NOTE 2: The adjustment angle is <b>not</b>automatically cleared whenever the sensor yaw
   * angle is reset.
   *
   * <p>If not set, the default adjustment angle is 0 degrees (no adjustment).
   *
   * @param adjustment, in degrees (range: -360 to 360)
   */
  public void setAngleAdjustment(double adjustment) {
    yaw_angle_tracker.setAngleAdjustment(adjustment);
  }

  /**
   * Returns the currently configured adjustment angle. See setAngleAdjustment() for more details.
   *
   * <p>If this method returns 0 degrees, no adjustment to the value returned via getAngle() will
   * occur.
   *
   * @return adjustment, in degrees (range: -360 to 360)
   */
  public double getAngleAdjustment() {
    return yaw_angle_tracker.getAngleAdjustment();
  }

  /**
   * Reset the Yaw gyro.
   *
   * <p>Resets the Gyro Z (Yaw) axis to a heading of zero. This can be used if there is significant
   * drift in the gyro and it needs to be recalibrated after it has been running.
   */
  public void reset() {
    zeroYaw();
  }

  private final float DEV_UNITS_MAX = 32768.0f;

  /**
   * Returns the current raw (unprocessed) X-axis gyro rotation rate (in degrees/sec). NOTE: this
   * value is un-processed, and should only be accessed by advanced users. Typically, rotation about
   * the X Axis is referred to as "Pitch". Calibrated and Integrated Pitch data is accessible via
   * the {@link #getPitch()} method.
   *
   * <p>
   *
   * @return Returns the current rotation rate (in degrees/sec).
   */
  public float getRawGyroX() {
    return this.raw_gyro_x / (DEV_UNITS_MAX / (float) gyro_fsr_dps);
  }

  /**
   * Returns the current raw (unprocessed) Y-axis gyro rotation rate (in degrees/sec). NOTE: this
   * value is un-processed, and should only be accessed by advanced users. Typically, rotation about
   * the T Axis is referred to as "Roll". Calibrated and Integrated Pitch data is accessible via the
   * {@link #getRoll()} method.
   *
   * <p>
   *
   * @return Returns the current rotation rate (in degrees/sec).
   */
  public float getRawGyroY() {
    return this.raw_gyro_y / (DEV_UNITS_MAX / (float) gyro_fsr_dps);
  }

  /**
   * Returns the current raw (unprocessed) Z-axis gyro rotation rate (in degrees/sec). NOTE: this
   * value is un-processed, and should only be accessed by advanced users. Typically, rotation about
   * the T Axis is referred to as "Yaw". Calibrated and Integrated Pitch data is accessible via the
   * {@link #getYaw()} method.
   *
   * <p>
   *
   * @return Returns the current rotation rate (in degrees/sec).
   */
  public float getRawGyroZ() {
    return this.raw_gyro_z / (DEV_UNITS_MAX / (float) gyro_fsr_dps);
  }

  /**
   * Returns the current raw (unprocessed) X-axis acceleration rate (in G). NOTE: this value is
   * unprocessed, and should only be accessed by advanced users. This raw value has not had
   * acceleration due to gravity removed from it, and has not been rotated to the world reference
   * frame. Gravity-corrected, world reference frame-corrected X axis acceleration data is
   * accessible via the {@link #getWorldLinearAccelX()} method.
   *
   * <p>
   *
   * @return Returns the current acceleration rate (in G).
   */
  public float getRawAccelX() {
    return this.raw_accel_x / (DEV_UNITS_MAX / (float) accel_fsr_g);
  }

  /**
   * Returns the current raw (unprocessed) Y-axis acceleration rate (in G). NOTE: this value is
   * unprocessed, and should only be accessed by advanced users. This raw value has not had
   * acceleration due to gravity removed from it, and has not been rotated to the world reference
   * frame. Gravity-corrected, world reference frame-corrected Y axis acceleration data is
   * accessible via the {@link #getWorldLinearAccelY()} method.
   *
   * <p>
   *
   * @return Returns the current acceleration rate (in G).
   */
  public float getRawAccelY() {
    return this.raw_accel_y / (DEV_UNITS_MAX / (float) accel_fsr_g);
  }

  /**
   * Returns the current raw (unprocessed) Z-axis acceleration rate (in G). NOTE: this value is
   * unprocessed, and should only be accessed by advanced users. This raw value has not had
   * acceleration due to gravity removed from it, and has not been rotated to the world reference
   * frame. Gravity-corrected, world reference frame-corrected Z axis acceleration data is
   * accessible via the {@link #getWorldLinearAccelZ()} method.
   *
   * <p>
   *
   * @return Returns the current acceleration rate (in G).
   */
  public float getRawAccelZ() {
    return this.raw_accel_z / (DEV_UNITS_MAX / (float) accel_fsr_g);
  }

  private final float UTESLA_PER_DEV_UNIT = 0.15f;

  /**
   * Returns the current raw (unprocessed) X-axis magnetometer reading (in uTesla). NOTE: this value
   * is unprocessed, and should only be accessed by advanced users. This raw value has not been
   * tilt-corrected, and has not been combined with the other magnetometer axis data to yield a
   * compass heading. Tilt-corrected compass heading data is accessible via the {@link
   * #getCompassHeading()} method.
   *
   * <p>
   *
   * @return Returns the mag field strength (in uTesla).
   */
  public float getRawMagX() {
    return this.cal_mag_x / UTESLA_PER_DEV_UNIT;
  }

  /**
   * Returns the current raw (unprocessed) Y-axis magnetometer reading (in uTesla). NOTE: this value
   * is unprocessed, and should only be accessed by advanced users. This raw value has not been
   * tilt-corrected, and has not been combined with the other magnetometer axis data to yield a
   * compass heading. Tilt-corrected compass heading data is accessible via the {@link
   * #getCompassHeading()} method.
   *
   * <p>
   *
   * @return Returns the mag field strength (in uTesla).
   */
  public float getRawMagY() {
    return this.cal_mag_y / UTESLA_PER_DEV_UNIT;
  }

  /**
   * Returns the current raw (unprocessed) Z-axis magnetometer reading (in uTesla). NOTE: this value
   * is unprocessed, and should only be accessed by advanced users. This raw value has not been
   * tilt-corrected, and has not been combined with the other magnetometer axis data to yield a
   * compass heading. Tilt-corrected compass heading data is accessible via the {@link
   * #getCompassHeading()} method.
   *
   * <p>
   *
   * @return Returns the mag field strength (in uTesla).
   */
  public float getRawMagZ() {
    return this.cal_mag_z / UTESLA_PER_DEV_UNIT;
  }

  /**
   * Returns the current barometric pressure (in millibar) [navX Aero only].
   *
   * <p>This value is valid only if a barometric pressure sensor is onboard.
   *
   * @return Returns the current barometric pressure (in millibar).
   */
  public float getPressure() {
    // TODO implement for navX-Aero.
    return 0;
  }

  /**
   * Returns the current temperature (in degrees centigrade) reported by the sensor's
   * gyro/accelerometer circuit.
   *
   * <p>This value may be useful in order to perform advanced temperature- correction of raw
   * gyroscope and accelerometer values.
   *
   * <p>
   *
   * @return The current temperature (in degrees centigrade).
   */
  public float getTempC() {
    return this.mpu_temp_c;
  }

  /**
   * Returns information regarding which sensor board axis (X,Y or Z) and direction (up/down) is
   * currently configured to report Yaw (Z) angle values. NOTE: If the board firmware supports
   * Omnimount, the board yaw axis/direction are configurable.
   *
   * <p>For more information on Omnimount, please see:
   *
   * <p>http://navx-mxp.kauailabs.com/navx-mxp/installation/omnimount/
   *
   * <p>
   *
   * @return The currently-configured board yaw axis/direction.
   */
  public BoardYawAxis getBoardYawAxis() {
    BoardYawAxis yaw_axis = new BoardYawAxis();
    short yaw_axis_info = (short) (capability_flags >> 3);
    yaw_axis_info &= 7;
    if (yaw_axis_info == AHRSProtocol.OMNIMOUNT_DEFAULT) {
      yaw_axis.up = true;
      yaw_axis.board_axis = BoardAxis.kBoardAxisZ;
    } else {
      yaw_axis.up = (((yaw_axis_info & 0x01) != 0) ? true : false);
      yaw_axis_info >>= 1;
      switch ((byte) yaw_axis_info) {
        case 0:
          yaw_axis.board_axis = BoardAxis.kBoardAxisX;
          break;
        case 1:
          yaw_axis.board_axis = BoardAxis.kBoardAxisY;
          break;
        case 2:
        default:
          yaw_axis.board_axis = BoardAxis.kBoardAxisZ;
          break;
      }
    }
    return yaw_axis;
  }

  /**
   * Returns the version number of the firmware currently executing on the sensor.
   *
   * <p>To update the firmware to the latest version, please see:
   *
   * <p>http://navx-mxp.kauailabs.com/navx-mxp/support/updating-firmware/
   *
   * <p>
   *
   * @return The firmware version in the format [MajorVersion].[MinorVersion]
   */
  public String getFirmwareVersion() {
    double version_number = (double) fw_ver_major;
    version_number += ((double) fw_ver_minor / 10);
    String fw_version = Double.toString(version_number);
    return fw_version;
  }

  /**
   * Enables or disables logging (via Console I/O) of AHRS library internal behaviors, including
   * events such as transient communication errors.
   *
   * @param enable true to enable logging; false to disable logging
   */
  public void enableLogging(boolean enable) {
    if (this.io != null) {
      io.enableLogging(enable);
    }
    this.logging_enabled = enable;
  }

  /**
   * Enables or disables board-level yaw zero (reset) requests. Board-level yaw resets are processed
   * by the sensor board and the resulting yaw angle may not be available to the client software
   * until at least 2 update cycles have occurred. Board-level yaw resets however do maintain
   * synchronization between the yaw angle and the sensor-generated Quaternion and Fused Heading
   * values.
   *
   * <p>Conversely, Software-based yaw resets occur instantaneously; however, Software- based yaw
   * resets do not update the yaw angle component of the sensor-generated Quaternion values or the
   * Fused Heading values.
   *
   * @param enable true to enable board-level yaw resets; false to enable software-based yaw resets.
   */
  public void enableBoardlevelYawReset(boolean enable) {
    enable_boardlevel_yawreset = enable;
  }

  /**
   * Returns true if Board-level yaw resets are enabled. Conversely, returns false if Software-based
   * yaw resets are active.
   *
   * @return true if Board-level yaw resets are enabled; false if software-based yaw resets are
   *     active.
   */
  public boolean isBoardlevelYawResetEnabled() {
    return enable_boardlevel_yawreset;
  }

  /**
   * Returns the sensor full scale range (in degrees per second) of the X, Y and X-axis gyroscopes.
   *
   * @return gyroscope full scale range in degrees/second.
   */
  public short getGyroFullScaleRangeDPS() {
    return this.gyro_fsr_dps;
  }

  /**
   * Returns the sensor full scale range (in G) of the X, Y and X-axis accelerometers.
   *
   * @return accelerometer full scale range in G.
   */
  public short getAccelFullScaleRangeG() {
    return this.accel_fsr_g;
  }

  /***********************************************************/
  /* Runnable Interface Implementation                       */
  /***********************************************************/

  class IOThread implements Runnable {

    Thread m_thread;
    boolean stop;

    public void start() {
      m_thread = new Thread(null, this, "navXIOThread");
      m_thread.start();
    }

    public void run() {
      io.run();
    }

    public void stop() {}
  }

  /***********************************************************/
  /* IBoardCapabilities Interface Implementation             */
  /***********************************************************/

  class BoardCapabilities implements IBoardCapabilities {

    @Override
    public boolean isOmniMountSupported() {
      return (((capability_flags & AHRSProtocol.NAVX_CAPABILITY_FLAG_OMNIMOUNT) != 0)
          ? true
          : false);
    }

    @Override
    public boolean isBoardYawResetSupported() {
      return (((capability_flags & AHRSProtocol.NAVX_CAPABILITY_FLAG_YAW_RESET) != 0)
          ? true
          : false);
    }

    @Override
    public boolean isDisplacementSupported() {
      return (((capability_flags & AHRSProtocol.NAVX_CAPABILITY_FLAG_VEL_AND_DISP) != 0)
          ? true
          : false);
    }

    @Override
    public boolean isAHRSPosTimestampSupported() {
      return (((capability_flags & AHRSProtocol.NAVX_CAPABILITY_FLAG_AHRSPOS_TS) != 0)
          ? true
          : false);
    }
  }

  /***********************************************************/
  /* IIOCompleteNotification Interface Implementation        */
  /***********************************************************/

  class IOCompleteNotification implements IIOCompleteNotification {

    @Override
    public void setYawPitchRoll(YPRUpdate ypr_update, long sensor_timestamp) {
      AHRS.this.yaw = ypr_update.yaw;
      AHRS.this.pitch = ypr_update.pitch;
      AHRS.this.roll = ypr_update.roll;
      AHRS.this.compass_heading = ypr_update.compass_heading;
      AHRS.this.last_sensor_timestamp = sensor_timestamp;
    }

    @Override
    public void setAHRSPosData(AHRSPosUpdate ahrs_update, long sensor_timestamp) {

      /* Update base IMU class variables */

      AHRS.this.yaw = ahrs_update.yaw;
      AHRS.this.pitch = ahrs_update.pitch;
      AHRS.this.roll = ahrs_update.roll;
      AHRS.this.compass_heading = ahrs_update.compass_heading;
      yaw_offset_tracker.updateHistory(ahrs_update.yaw);

      /* Update AHRS class variables */

      // 9-axis data
      AHRS.this.fused_heading = ahrs_update.fused_heading;

      // Gravity-corrected linear acceleration (world-frame)
      AHRS.this.world_linear_accel_x = ahrs_update.linear_accel_x;
      AHRS.this.world_linear_accel_y = ahrs_update.linear_accel_y;
      AHRS.this.world_linear_accel_z = ahrs_update.linear_accel_z;

      // Gyro/Accelerometer Die Temperature
      AHRS.this.mpu_temp_c = ahrs_update.mpu_temp;

      // Barometric Pressure/Altitude
      AHRS.this.altitude = ahrs_update.altitude;
      AHRS.this.baro_pressure = ahrs_update.barometric_pressure;

      // Status/Motion Detection
      AHRS.this.is_moving =
          (((ahrs_update.sensor_status & AHRSProtocol.NAVX_SENSOR_STATUS_MOVING) != 0)
              ? true
              : false);
      AHRS.this.is_rotating =
          (((ahrs_update.sensor_status & AHRSProtocol.NAVX_SENSOR_STATUS_YAW_STABLE) != 0)
              ? false
              : true);
      AHRS.this.altitude_valid =
          (((ahrs_update.sensor_status & AHRSProtocol.NAVX_SENSOR_STATUS_ALTITUDE_VALID) != 0)
              ? true
              : false);
      AHRS.this.is_magnetometer_calibrated =
          (((ahrs_update.cal_status & AHRSProtocol.NAVX_CAL_STATUS_MAG_CAL_COMPLETE) != 0)
              ? true
              : false);
      AHRS.this.magnetic_disturbance =
          (((ahrs_update.sensor_status & AHRSProtocol.NAVX_SENSOR_STATUS_MAG_DISTURBANCE) != 0)
              ? true
              : false);

      AHRS.this.quaternionW = ahrs_update.quat_w;
      AHRS.this.quaternionX = ahrs_update.quat_x;
      AHRS.this.quaternionY = ahrs_update.quat_y;
      AHRS.this.quaternionZ = ahrs_update.quat_z;

      AHRS.this.last_sensor_timestamp = sensor_timestamp;

      velocity[0] = ahrs_update.vel_x;
      velocity[1] = ahrs_update.vel_y;
      velocity[2] = ahrs_update.vel_z;
      displacement[0] = ahrs_update.disp_x;
      displacement[1] = ahrs_update.disp_y;
      displacement[2] = ahrs_update.disp_z;

      updateBoardStatus(
          ahrs_update.op_status,
          ahrs_update.sensor_status,
          ahrs_update.cal_status,
          ahrs_update.selftest_status);

      yaw_angle_tracker.nextAngle(getYaw());

      /* Notify external data arrival subscribers, if any. */
      for (int i = 0; i < callbacks.length; i++) {
        ITimestampedDataSubscriber callback = callbacks[i];
        if (callback != null) {
          long system_timestamp = (long) (Timer.getFPGATimestamp() * 1000);
          callback.timestampedDataReceived(
              system_timestamp, sensor_timestamp, ahrs_update, callback_contexts[i]);
        }
      }
    }

    @Override
    public void setRawData(AHRSProtocol.GyroUpdate raw_data_update, long sensor_timestamp) {
      AHRS.this.raw_gyro_x = raw_data_update.gyro_x;
      AHRS.this.raw_gyro_y = raw_data_update.gyro_y;
      AHRS.this.raw_gyro_z = raw_data_update.gyro_z;
      AHRS.this.raw_accel_x = raw_data_update.accel_x;
      AHRS.this.raw_accel_y = raw_data_update.accel_y;
      AHRS.this.raw_accel_z = raw_data_update.accel_z;
      AHRS.this.cal_mag_x = raw_data_update.mag_x;
      AHRS.this.cal_mag_y = raw_data_update.mag_y;
      AHRS.this.cal_mag_z = raw_data_update.mag_z;
      AHRS.this.mpu_temp_c = raw_data_update.temp_c;

      AHRS.this.last_sensor_timestamp = sensor_timestamp;
    }

    @Override
    public void setAHRSData(AHRSProtocol.AHRSUpdate ahrs_update, long sensor_timestamp) {

      /* Update base IMU class variables */

      AHRS.this.yaw = ahrs_update.yaw;
      AHRS.this.pitch = ahrs_update.pitch;
      AHRS.this.roll = ahrs_update.roll;
      AHRS.this.compass_heading = ahrs_update.compass_heading;
      yaw_offset_tracker.updateHistory(ahrs_update.yaw);

      /* Update AHRS class variables */

      // 9-axis data
      AHRS.this.fused_heading = ahrs_update.fused_heading;

      // Gravity-corrected linear acceleration (world-frame)
      AHRS.this.world_linear_accel_x = ahrs_update.linear_accel_x;
      AHRS.this.world_linear_accel_y = ahrs_update.linear_accel_y;
      AHRS.this.world_linear_accel_z = ahrs_update.linear_accel_z;

      // Gyro/Accelerometer Die Temperature
      AHRS.this.mpu_temp_c = ahrs_update.mpu_temp;

      // Barometric Pressure/Altitude
      AHRS.this.altitude = ahrs_update.altitude;
      AHRS.this.baro_pressure = ahrs_update.barometric_pressure;

      // Magnetometer Data
      AHRS.this.cal_mag_x = ahrs_update.cal_mag_x;
      AHRS.this.cal_mag_y = ahrs_update.cal_mag_y;
      AHRS.this.cal_mag_z = ahrs_update.cal_mag_z;

      // Status/Motion Detection
      AHRS.this.is_moving =
          (((ahrs_update.sensor_status & AHRSProtocol.NAVX_SENSOR_STATUS_MOVING) != 0)
              ? true
              : false);
      AHRS.this.is_rotating =
          (((ahrs_update.sensor_status & AHRSProtocol.NAVX_SENSOR_STATUS_YAW_STABLE) != 0)
              ? false
              : true);
      AHRS.this.altitude_valid =
          (((ahrs_update.sensor_status & AHRSProtocol.NAVX_SENSOR_STATUS_ALTITUDE_VALID) != 0)
              ? true
              : false);
      AHRS.this.is_magnetometer_calibrated =
          (((ahrs_update.cal_status & AHRSProtocol.NAVX_CAL_STATUS_MAG_CAL_COMPLETE) != 0)
              ? true
              : false);
      AHRS.this.magnetic_disturbance =
          (((ahrs_update.sensor_status & AHRSProtocol.NAVX_SENSOR_STATUS_MAG_DISTURBANCE) != 0)
              ? true
              : false);

      AHRS.this.quaternionW = ahrs_update.quat_w;
      AHRS.this.quaternionX = ahrs_update.quat_x;
      AHRS.this.quaternionY = ahrs_update.quat_y;
      AHRS.this.quaternionZ = ahrs_update.quat_z;

      AHRS.this.last_sensor_timestamp = sensor_timestamp;

      updateBoardStatus(
          ahrs_update.op_status,
          ahrs_update.sensor_status,
          ahrs_update.cal_status,
          ahrs_update.selftest_status);

      updateDisplacement(
          AHRS.this.world_linear_accel_x,
          AHRS.this.world_linear_accel_y,
          update_rate_hz,
          AHRS.this.is_moving);

      yaw_angle_tracker.nextAngle(getYaw());

      /* Notify external data arrival subscribers, if any. */
      for (int i = 0; i < callbacks.length; i++) {
        ITimestampedDataSubscriber callback = callbacks[i];
        if (callback != null) {
          long system_timestamp = (long) (Timer.getFPGATimestamp() * 1000);
          callback.timestampedDataReceived(
              system_timestamp, sensor_timestamp, ahrs_update, callback_contexts[i]);
        }
      }
    }

    @Override
    public void setBoardID(BoardID board_id) {
      board_type = board_id.type;
      hw_rev = board_id.hw_rev;
      fw_ver_major = board_id.fw_ver_major;
      fw_ver_minor = board_id.fw_ver_minor;
      String boardtype = "unknown";
      if (board_id.type == 50) {
        boardtype = "navX-Sensor";
      }
      if (board_id.hw_rev == 33) {
        boardtype = "navX-MXP (Classic)";
      } else if (board_id.hw_rev == 34) {
        boardtype = "navX2-MXP (Gen 2)";
      } else if (board_id.hw_rev == 40) {
        boardtype = "navX-Micro (Classic)";
      } else if (board_id.hw_rev == 41) {
        boardtype = "navX2-Micro (Gen 2)";
      } else if ((board_id.hw_rev >= 60) && (board_id.hw_rev <= 69)) {
        boardtype = "VMX-pi";
      }
      Tracer.Trace("navX-Sensor Board Type %d (%s)\n", board_id.type, boardtype);
      Tracer.Trace(
          "navX-Sensor firmware version %d.%d\n", board_id.fw_ver_major, board_id.fw_ver_minor);
    }

    @Override
    public void setBoardState(BoardState board_state, boolean update_board_status) {
      update_rate_hz = board_state.update_rate_hz;
      accel_fsr_g = board_state.accel_fsr_g;
      gyro_fsr_dps = board_state.gyro_fsr_dps;
      capability_flags = board_state.capability_flags;
      if (update_board_status) {
        updateBoardStatus(
            board_state.op_status,
            board_state.sensor_status,
            board_state.cal_status,
            board_state.selftest_status);
      }
    }

    void updateBoardStatus(
        byte op_status, short sensor_status, byte cal_status, byte selftest_status) {
      /* Detect/Report Board operational status transitions */
      boolean poweron_init_completed = false;
      if (AHRS.this.op_status == AHRSProtocol.NAVX_OP_STATUS_NORMAL) {
        if (op_status != AHRSProtocol.NAVX_OP_STATUS_NORMAL) {
          /* Board reset detected */
          Tracer.Trace("navX-Sensor Reset Detected.\n");
        }
      } else {
        if (op_status == AHRSProtocol.NAVX_OP_STATUS_NORMAL) {
          poweron_init_completed = true;
          if ((cal_status & AHRSProtocol.NAVX_CAL_STATUS_IMU_CAL_STATE_MASK)
              != AHRSProtocol.NAVX_CAL_STATUS_IMU_CAL_COMPLETE) {
            Tracer.Trace(
                "navX-Sensor startup initialization underway; startup calibration in progress.\n");
          } else {
            // Tracer.Trace("navX-Sensor startup initialization and startup calibration
            // complete.\n");
          }
        }
      }

      /* Detect/report reset of yaw angle tracker upon transition to startup calibration complete state. */
      if (((AHRS.this.cal_status & AHRSProtocol.NAVX_CAL_STATUS_IMU_CAL_STATE_MASK)
              != AHRSProtocol.NAVX_CAL_STATUS_IMU_CAL_COMPLETE)
          && ((cal_status & AHRSProtocol.NAVX_CAL_STATUS_IMU_CAL_STATE_MASK)
              == AHRSProtocol.NAVX_CAL_STATUS_IMU_CAL_COMPLETE)) {
        Tracer.Trace("navX-Sensor onboard startup calibration complete." + "\n");
        /* Carefully, only reset the software yaw reset offset if calibration completion upon */
        /* initial poweron or after board reset occurs. */
        if (poweron_init_completed || AHRS.this.disconnect_startupcalibration_recovery_pending) {
          AHRS.this.disconnect_startupcalibration_recovery_pending = false;
          AHRS.this.yaw_angle_tracker.init();
          Tracer.Trace("navX-Sensor Yaw angle auto-reset to 0.0 due to startup calibration.\n");
        }
      }

      AHRS.this.op_status = op_status;
      AHRS.this.sensor_status = sensor_status;
      AHRS.this.cal_status = cal_status;
      AHRS.this.selftest_status = selftest_status;
    }

    @Override
    public void yawResetComplete() {
      AHRS.this.yaw_angle_tracker.reset();
      if (AHRS.this.enable_boardlevel_yawreset) {
        Tracer.Trace("navX-Sensor Board-level Yaw Reset completed.\n");
      } else {
        Tracer.Trace("navX-Sensor Software Yaw Reset completed.\n");
      }
    }

    @Override
    public void disconnectDetected() {
      /* Board disconnect may be caused by intermittent communication loss, or by board reset. */
      /* Default status to error */
      AHRS.this.op_status = AHRSProtocol.NAVX_OP_STATUS_ERROR;
      AHRS.this.sensor_status = 0; /* Clear all sensor status flags */
      /* Flag the need to watch for a startup calibration completion event upon later reconnect. */
      AHRS.this.disconnect_startupcalibration_recovery_pending = true;
      Tracer.Trace("navX-Sensor DISCONNECTED!!!.\n");
    }

    @Override
    public void connectDetected() {
      Tracer.Trace("navX-Sensor Connected.\n");
    }
  }
  ;

  /***********************************************************/
  /* Sendable Interface Implementation             */
  /***********************************************************/

  /**
   * Initializes smart dashboard communication.
   *
   * @param builder The SendableBuilder which will be registered with.
   */
  @Override
  public void initSendable(NTSendableBuilder builder) {
    builder.setSmartDashboardType("Gyro");
    builder.addDoubleProperty("Value", this::getYaw, null);
  }

  /***********************************************************/
  /* AutoCloseable Interface Implementation                  */
  /***********************************************************/
  @Override
  public void close() {
    SendableRegistry.remove(this);
    if (io != null) {
      io.stop();
    }
    /* Note:  Currently this method does not release the underlying port resource. */
    if (m_simDevice != null) {
      m_simDevice.close();
      m_simDevice = null;
    }
  }

  /************************************************************/
  /* Gyro interface Implementation                            */
  /************************************************************/
  public void calibrate() {}
}
