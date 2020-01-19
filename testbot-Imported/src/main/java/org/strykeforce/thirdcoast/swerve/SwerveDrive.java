package org.strykeforce.thirdcoast.swerve;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.thirdcoast.talon.Errors;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Robot;
import frc.robot.control.DriverControls;

/**
 * Control a Third Coast swerve drive.
 *
 * <p>Wheels are a array numbered 0-3 from front to back, with even numbers on the left side when
 * facing forward.
 *
 * <p>Derivation of inverse kinematic equations are from Ether's <a
 * href="https://www.chiefdelphi.com/media/papers/2426">Swerve Kinematics and Programming</a>.
 *
 * @see Wheel
 */
@SuppressWarnings("unused")
public class SwerveDrive {

  public static final int DEFAULT_ABSOLUTE_AZIMUTH_OFFSET = 200;
  private static final Logger logger = LoggerFactory.getLogger(SwerveDrive.class);
  private static final int WHEEL_COUNT = 4;
  private final AHRS gyro;
  private final double kLengthComponent;
  private final double kWidthComponent;
  private final double kGyroRateCorrection;
  private final Wheel[] wheels;
  private final double[] ws = new double[WHEEL_COUNT];
  private final double[] wa = new double[WHEEL_COUNT];
  private boolean isFieldOriented;
  private CANSparkMax m_motor;

  public SwerveDrive(SwerveDriveConfig config) {
    //logger.info("<b>SwerveDrive</b>: SwerveDrive starting");
    m_motor = new CANSparkMax(0, MotorType.kBrushless);
    m_motor.disable();

    gyro = config.gyro;
    wheels = config.wheels;

    final boolean summarizeErrors = config.summarizeTalonErrors;
    Errors.setSummarized(summarizeErrors);
    Errors.setCount(0);
    //logger.debug("TalonSRX configuration errors summarized = {}", summarizeErrors);

    double length = config.length;
    double width = config.width;
    double radius = Math.hypot(length, width);
    kLengthComponent = length / radius;
    kWidthComponent = width / radius;

    SmartDashboard.putString("Field Oriented", "nyet");

    logger.info("gyro is configured: {}", gyro != null);
    logger.info("gyro is connected: {}", gyro != null && gyro.isConnected());
    SmartDashboard.putString("gyro is connected", (gyro != null && gyro.isConnected()) ? "hm yes" : "uh no");
    setFieldOriented(gyro != null && gyro.isConnected());

    UpdateGyroDashboard();
    
    if (isFieldOriented) {
      logger.info("ROBOT IS FIELD ORIENTED IN SWERVEDRIVE.JAVA");
      gyro.enableLogging(config.gyroLoggingEnabled);
      double robotPeriod = config.robotPeriod;
      double gyroRateCoeff = config.gyroRateCoeff;
      int rate = gyro.getActualUpdateRate();
      double gyroPeriod = 1.0 / rate;
      kGyroRateCorrection = (robotPeriod / gyroPeriod) * gyroRateCoeff;
      logger.debug("gyro frequency = {} Hz", rate);
    } else {
      logger.warn("gyro is missing or not enabled");
      kGyroRateCorrection = 0;
    }

    logger.debug("length = {}", length);
    logger.debug("width = {}", width);
    logger.debug("enableGyroLogging = {}", config.gyroLoggingEnabled);
    logger.debug("gyroRateCorrection = {}", kGyroRateCorrection);

    //logger.info("<b>SwerveDrive</b>: SwerveDrive constructed");
  }

  private void UpdateGyroDashboard(){
    if(gyro != null){
      SmartDashboard.putNumber("Gyro Orientation", gyro.getAngle());
    } else {
     SmartDashboard.putNumber("Gyro Orientation", 361);
    }

  }

  /**
   * Return key that wheel zero information is stored under in WPI preferences.
   *
   * @param wheel the wheel number
   * @return the String key
   */
  public static String getPreferenceKeyForWheel(int wheel) {
    return String.format("%s/wheel.%d", SwerveDrive.class.getSimpleName(), wheel);
  }

  /**
   * Set the drive mode.
   *
   * @param driveMode the drive mode
   */
  public void setDriveMode(DriveMode driveMode) {
    //logger.info("<b>SwerveDrive</b>: setDriveMode starting");
    for (Wheel wheel : wheels) {
      wheel.setDriveMode(driveMode);
    }
    logger.info("drive mode = {}", driveMode);
    logger.info("gyro is configured: {}", gyro != null);
    logger.info("gyro is connected: {}", gyro != null && gyro.isConnected());
    SmartDashboard.putString("gyro is connected", (gyro != null && gyro.isConnected()) ? "hm yes" : "uh no");
    if (isFieldOriented) {
      logger.info("ROBOT IS FIELD ORIENTED IN setDriveMode()");
    }
    //logger.info("<b>SwerveDrive</b>: setDriveMode finished");
  }

  /**
   * Set all four wheels to specified values.
   *
   * @param azimuth -0.5 to 0.5 rotations, measured clockwise with zero being the robot
   *     straight-ahead position
   * @param drive 0 to 1 in the direction of the wheel azimuth
   */
  public void set(double azimuth, double drive) {
    //logger.info("<b>SwerveDrive</b>: set starting");
    for (Wheel wheel : wheels) {
      wheel.set(azimuth, drive);
    }
    //logger.info("<b>SwerveDrive</b>: set finished");
  }

  /**
   * Drive the robot in given field-relative direction and with given rotation.
   *
   * @param forward Y-axis movement, from -1.0 (reverse) to 1.0 (forward)
   * @param strafe X-axis movement, from -1.0 (left) to 1.0 (right)
   * @param azimuth robot rotation, from -1.0 (CCW) to 1.0 (CW)
   */
  public void drive(double forward, double strafe, double azimuth) {
    //logger.info("<b>SwerveDrive</b>: drive starting");
    UpdateGyroDashboard();
    // Use gyro for field-oriented drive. We use getAngle instead of getYaw to enable arbitrary
    // autonomous starting positions.
    if (isFieldOriented) {
      logger.info("ROBOT IS FIELD ORIENTED");
      double angle = gyro.getAngle();
      SmartDashboard.putNumber("GYRO_ANGLE", angle);
      SmartDashboard.putNumber("GYRO_RATE", gyro.getRate());
      angle += gyro.getRate() * kGyroRateCorrection;
      angle = Math.IEEEremainder(angle, 360.0);

      angle = Math.toRadians(angle);
      final double temp = forward * Math.cos(angle) + strafe * Math.sin(angle);
      strafe = strafe * Math.cos(angle) - forward * Math.sin(angle);
      forward = temp;
    }

    final double a = strafe - azimuth * kLengthComponent;
    final double b = strafe + azimuth * kLengthComponent;
    final double c = forward - azimuth * kWidthComponent;
    final double d = forward + azimuth * kWidthComponent;

    // wheel speed
    ws[0] = Math.hypot(b, d);
    ws[1] = Math.hypot(b, c);
    ws[2] = Math.hypot(a, d);
    ws[3] = Math.hypot(a, c);

    // wheel azimuth
    wa[0] = Math.atan2(b, d) * 0.5 / Math.PI;
    wa[1] = Math.atan2(b, c) * 0.5 / Math.PI;
    wa[2] = Math.atan2(a, d) * 0.5 / Math.PI;
    wa[3] = Math.atan2(a, c) * 0.5 / Math.PI;

    // normalize wheel speed
    final double maxWheelSpeed = Math.max(Math.max(ws[0], ws[1]), Math.max(ws[2], ws[3]));
    if (maxWheelSpeed > 1.0) {
      for (int i = 0; i < WHEEL_COUNT; i++) {
        ws[i] /= maxWheelSpeed;
      }
    }

    // set wheels
    for (int i = 0; i < WHEEL_COUNT; i++) {
      wheels[i].set(wa[i], ws[i]);
    }
    //logger.info("<b>SwerveDrive</b>: drive finished");
  }

  /**
   * Stops all wheels' azimuth and drive movement. Calling this in the robots {@code teleopInit} and
   * {@code autonomousInit} will reset wheel azimuth relative encoders to the current position and
   * thereby prevent wheel rotation if the wheels were moved manually while the robot was disabled.
   */
  public void stop() {
    //logger.info("<b>SwerveDrive</b>: stop starting");
    for (Wheel wheel : wheels) {
      wheel.stop();
    }
    logger.info("stopped all wheels");

    //logger.info("<b>SwerveDrive</b>: stop finished");
  }

  /**
   * Save the wheels' azimuth current position as read by absolute encoder. These values are saved
   * persistently on the roboRIO and are normally used to calculate the relative encoder offset
   * during wheel initialization.
   *
   * <p>The wheel alignment data is saved in the WPI preferences data store and may be viewed using
   * a network tables viewer.
   *
   * @see #zeroAzimuthEncoders()
   */
  public void saveAzimuthPositions() {
    saveAzimuthPositions(Preferences.getInstance());
  }

  void saveAzimuthPositions(Preferences prefs) {
    //logger.info("<b>SwerveDrive</b>: saveAzimuthPositions starting");
    for (int i = 0; i < WHEEL_COUNT; i++) {
      int position = wheels[i].getAzimuthAbsolutePosition();
      prefs.putInt(getPreferenceKeyForWheel(i), position);
      logger.info("azimuth {}: saved zero = {}", i, position);
    }
    //logger.info("<b>SwerveDrive</b>: saveAzimuthPositions finished");
  }

  /**
   * Set wheels' azimuth relative offset from zero based on the current absolute position. This uses
   * the physical zero position as read by the absolute encoder and saved during the wheel alignment
   * process.
   *
   * @see #saveAzimuthPositions()
   */
  public void zeroAzimuthEncoders() {
    zeroAzimuthEncoders(Preferences.getInstance());
  }

  void zeroAzimuthEncoders(Preferences prefs) {
    //logger.info("<b>SwerveDrive</b>: zeroAzimuthEncoders starting");
    Errors.setCount(0);
    for (int i = 0; i < WHEEL_COUNT; i++) {
      int position = prefs.getInt(getPreferenceKeyForWheel(i), DEFAULT_ABSOLUTE_AZIMUTH_OFFSET);
      wheels[i].setAzimuthZero(position);
      logger.info("azimuth {}: loaded zero = {}", i, position);
    }
    int errorCount = Errors.getCount();
    if (errorCount > 0) logger.error("TalonSRX set azimuth zero error count = {}", errorCount);

    //logger.info("<b>SwerveDrive</b>: zeroAzimuthEncoders finished");
  }

  /**
   * Returns the four wheels of the swerve drive.
   *
   * @return the Wheel array.
   */
  public Wheel[] getWheels() {
    return wheels;
  }

  /**
   * Get the gyro instance being used by the drive.
   *
   * @return the gyro instance
   */
  public AHRS getGyro() {
    return gyro;
  }

  /**
   * Get status of field-oriented driving.
   *
   * @return status of field-oriented driving.
   */
  public boolean isFieldOriented() {
    return isFieldOriented;
  }

  /**
   * Enable or disable field-oriented driving. Enabled by default if connected gyro is passed in via
   * {@code SwerveDriveConfig} during construction.
   *
   * @param enabled true to enable field-oriented driving.
   */
  public void setFieldOriented(boolean enabled) {
    isFieldOriented = enabled;
    SmartDashboard.putString("Field Oriented", enabled ? "da" : "nyet");
    logger.info("field orientation driving is {}", isFieldOriented ? "ENABLED" : "DISABLED");
  }

  /**
   * Unit testing
   *
   * @return length
   */
  double getLengthComponent() {
    return kLengthComponent;
  }

  /**
   * Unit testing
   *
   * @return width
   */
  double getWidthComponent() {
    return kWidthComponent;
  }

  /** Swerve Drive drive mode */
  public enum DriveMode {
    OPEN_LOOP,
    CLOSED_LOOP,
    TELEOP,
    TRAJECTORY,
    AZIMUTH
  }
}
