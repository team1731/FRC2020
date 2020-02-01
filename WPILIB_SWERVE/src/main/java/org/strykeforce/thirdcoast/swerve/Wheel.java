package org.strykeforce.thirdcoast.swerve;

import static com.ctre.phoenix.motorcontrol.ControlMode.MotionMagic;
import static com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput;
import static com.ctre.phoenix.motorcontrol.ControlMode.Velocity;
import static org.strykeforce.thirdcoast.swerve.SwerveDrive.DriveMode.TELEOP;

import java.util.Objects;
import java.util.function.DoubleConsumer;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.ControlType;
//import com.sun.java.swing.plaf.windows.TMSchema.Control;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.thirdcoast.swerve.SwerveDrive.DriveMode;
import org.strykeforce.thirdcoast.talon.Errors;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Controls a swerve drive wheel azimuth and drive motors.
 *
 * <p>
 * The swerve-drive inverse kinematics algorithm will always calculate
 * individual wheel angles as -0.5 to 0.5 rotations, measured clockwise with
 * zero being the straight-ahead position. Wheel speed is calculated as 0 to 1
 * in the direction of the wheel angle.
 *
 * <p>
 * This class will calculate how to implement this angle and drive direction
 * optimally for the azimuth and drive motors. In some cases it makes sense to
 * reverse wheel direction to avoid rotating the wheel azimuth 180 degrees.
 *
 * <p>
 * Hardware assumed by this class includes a CTRE magnetic encoder on the
 * azimuth motor and no limits on wheel azimuth rotation. Azimuth Talons have an
 * ID in the range 0-3 with corresponding drive Talon IDs in the range 10-13.
 */
public class Wheel {
  public int wheelID;
  private static final int TICKS = 16;

  private static final Logger logger = LoggerFactory.getLogger(Wheel.class);
  private final double driveSetpointMax;
  //private final TalonSRX driveTalon;
  private final CANSparkMax driveSpark;
  //private final TalonSRX azimuthTalon;
  private final CANSparkMax azimuthSpark;
  protected DoubleConsumer driver;
  private boolean isInverted = false;
  private CANPIDController m_pidController;
  public CANEncoder m_encoder;


  /**
   * This constructs a wheel with supplied azimuth and drive talons.
   *
   * <p>Wheels will scale closed-loop drive output to {@code driveSetpointMax}. For example, if
   * closed-loop drive mode is tuned to have a max usable output of 10,000 ticks per 100ms, set this
   * to 10,000 and the wheel will send a setpoint of 10,000 to the drive talon when wheel is set to
   * max drive output (1.0).
   *
   * @param azimuth the configured azimuth TalonSRX
   * @param drive the configured drive TalonSRX
   * @param driveSetpointMax scales closed-loop drive output to this value when drive setpoint = 1.0
   */
  public Wheel(CANSparkMax azimuth, CANSparkMax drive, double driveSetpointMax) {
    //logger.info("<b>Wheel</b>: Wheel starting");
    this.driveSetpointMax = driveSetpointMax;
    azimuthSpark = Objects.requireNonNull(azimuth);
    driveSpark = Objects.requireNonNull(drive);
    m_pidController = azimuthSpark.getPIDController();
    m_encoder = azimuthSpark.getEncoder();

    setDriveMode(TELEOP);

    //logger.debug("azimuth = {} drive = {}", azimuthTalon.getDeviceID(), driveTalon.getDeviceID());
    logger.info("azimuth = {} drive = {}", azimuthSpark.getDeviceId(), driveSpark.getDeviceId());
    logger.info("driveSetpointMax = {}", driveSetpointMax);
    if (driveSetpointMax == 0.0) logger.warn("driveSetpointMax may not have been configured");

    //logger.info("<b>Wheel</b>: Wheel constructed");
  }

  /**
   * This method calculates the optimal driveTalon settings and applies them.
   *
   * @param azimuth -0.5 to 0.5 rotations, measured clockwise with zero being the wheel's zeroed
   *     position
   * @param drive 0 to 1.0 in the direction of the wheel azimuth
   */
  public void set(double azimuth, double drive) {
   // logger.info("<b>Wheel</b>: set starting");
    // don't reset wheel azimuth direction to zero when returning to neutral
    SmartDashboard.putNumber("Wheel"+wheelID+" Encoder Pos", m_encoder.getPosition());
    SmartDashboard.putNumber("Wheel"+wheelID+" Encoder Get Abs", getAzimuthAbsolutePosition());
    if (drive == 0) {
      //logger.info("<b>Wheel</b>: drive == 0. Going through anyway.");
      //logger.info("<b>Wheel</b>: set returning. drive == 0");
      //driver.accept(0d);
      //return;
    }

    azimuth *= TICKS; // flip azimuth, hardware configuration dependent

    SmartDashboard.putNumber("Wheel"+wheelID+" BDL wanted AZ in ticks",azimuth);

    //double azimuthPosition = azimuthTalon.getSelectedSensorPosition(0);


    double azimuthPosition = m_encoder.getPosition();
    double azimuthError = Math.IEEEremainder(azimuth - azimuthPosition, TICKS);
    
    SmartDashboard.putNumber("Wheel"+wheelID+" BDL actual AZ in ticks",azimuthPosition);
    SmartDashboard.putNumber("Wheel"+wheelID+" BDL actual error in ticks",azimuthError);


    // minimize azimuth rotation, reversing drive if necessary
    isInverted = Math.abs(azimuthError) > 0.25 * TICKS;
    if (isInverted) {
      azimuthError -= Math.copySign(0.5 * TICKS, azimuthError);
      drive = -drive;
    }

    //azimuthTalon.set(MotionMagic, azimuthPosition + azimuthError);
    //azimuthSpark.set(azimuthPosition + azimuthError);
    SmartDashboard.putNumber("Wheel"+wheelID+" BDL Commanded Position",(azimuthPosition + azimuthError));
    m_pidController.setReference((azimuthPosition + azimuthError), ControlType.kSmartMotion);
    //m_pidController.setReference(5, ControlType.kSmartMotion);
    //driver.accept(drive);
    driveSpark.getPIDController().setReference(drive, ControlType.kVelocity);
  //  logger.info("<b>Wheel</b>: set finished");
  }

  /**
   * Set azimuth to encoder position.
   *
   * @param position position in encoder ticks.
   */
  public void setAzimuthPosition(int position) {
    //logger.info("<b>Wheel</b>: setAzimuthPosition starting");
    //azimuthTalon.set(MotionMagic, position);
    //azimuthSpark.set(position);
    m_pidController.setReference(position, ControlType.kSmartMotion);
    //logger.info("<b>Wheel</b>: setAzimuthPosition finished");
  }

  public void disableAzimuth() {
    //logger.info("<b>Wheel</b>: disableAzimuth starting");
    //azimuthTalon.neutralOutput();
    azimuthSpark.set(0);
    //logger.info("<b>Wheel</b>: disableAzimuth finished");
  }

  /**
   * Set the operating mode of the wheel's drive motors. In this default wheel implementation {@code
   * OPEN_LOOP} and {@code TELEOP} are equivalent and {@code CLOSED_LOOP}, {@code TRAJECTORY} and
   * {@code AZIMUTH} are equivalent.
   *
   * <p>In closed-loop modes, the drive setpoint is scaled by the drive Talon {@code
   * driveSetpointMax} parameter.
   *
   * <p>This method is intended to be overridden if the open or closed-loop drive wheel drivers need
   * to be customized.
   *
   * @param driveMode the desired drive mode
   */
  public void setDriveMode(DriveMode driveMode) {
    //logger.info("<b>Wheel</b>: setDriveMode starting");
    switch (driveMode) {
      case OPEN_LOOP:
      case TELEOP:
        //driver = (setpoint) -> driveTalon.set(PercentOutput, setpoint);
        driver = (setpoint) -> driveSpark.set(setpoint);
      break;
      case CLOSED_LOOP:
      case TRAJECTORY:
      case AZIMUTH:
        //driver = (setpoint) -> driveTalon.set(Velocity, setpoint * driveSetpointMax);
        driver = (setpoint) -> m_pidController.setReference(setpoint * driveSetpointMax, ControlType.kSmartMotion);
        //driver = (setpoint) -> driveSpark.set(setpoint * driveSetpointMax);
        break;
    }
    //logger.info("<b>Wheel</b>: setDriveMode finished");
  }

  /**
   * Stop azimuth and drive movement. This resets the azimuth setpoint and relative encoder to the
   * current position in case the wheel has been manually rotated away from its previous setpoint.
   */
  public void stop() {
    //logger.info("<b>Wheel</b>: stop starting");
    //azimuthTalon.set(MotionMagic, azimuthTalon.getSelectedSensorPosition(0));
    //azimuthSpark.set(azimuthSpark.get());
    m_pidController.setReference(getAzimuthAbsolutePosition(), ControlType.kSmartMotion);
    driver.accept(0d);
    //logger.info("<b>Wheel</b>: stop finished");
  }

  /**
   * Set the azimuthTalon encoder relative to wheel zero alignment position. For example, if current
   * absolute encoder = 0 and zero setpoint = 2767, then current relative setpoint = -2767.
   *
   * <pre>
   *
   * relative:  -2767                               0
   *           ---|---------------------------------|-------
   * absolute:    0                               2767
   *
   * </pre>
   *
   * @param zero zero setpoint, absolute encoder position (in ticks) where wheel is zeroed.
   */
  public void setAzimuthZero(int zero) {
    //logger.info("<b>Wheel</b>: setAzimuthZero starting");
    int azimuthSetpoint = getAzimuthAbsolutePosition() - zero;
    //ErrorCode err = azimuthTalon.setSelectedSensorPosition(azimuthSetpoint, 0, 10);
    //Errors.check(err, logger);
    //azimuthTalon.set(MotionMagic, azimuthSetpoint);
    //azimuthSpark.set(azimuthSetpoint);
    //m_pidController.setReference(azimuthSetpoint, ControlType.kSmartMotion);
    m_encoder.setPosition(0);  //TODO change whith 221 encoder
    //logger.info("<b>Wheel</b>: setAzimuthZero finished");
  }

  /**
   * Returns the wheel's azimuth absolute position in encoder ticks.
   *
   * @return 0 - 4095, corresponding to one full revolution.
   */
  public int getAzimuthAbsolutePosition() {
    //return azimuthTalon.getSensorCollection().getPulseWidthPosition() & 0xFFF;
    //return (int)azimuthSpark.get() & 0xFFF;
    //TODO - need to return azimuth from the 221 encoder
    return 0;
  }

  /**
   * Get the azimuth Talon controller.
   *
   * @return azimuth Talon instance used by wheel
   */
  public CANSparkMax getAzimuthTalon() {
    return azimuthSpark;
  }

  /**
   * Get the drive Talon controller.
   *
   * @return drive Talon instance used by wheel
   */
  public CANSparkMax getDriveTalon() {
    return driveSpark;
  }

  public double getDriveSetpointMax() {
    return driveSetpointMax;
  }

  public boolean isInverted() {
    return isInverted;
  }

  @Override
  public String toString() {
    return "Wheel{"
        + "azimuthSpark="
        + azimuthSpark
        + ", driveSpark="
        + driveSpark
        + ", driveSetpointMax="
        + driveSetpointMax
        + '}';
  }
}
