/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


//import edu.wpi.first.wpilibj.Encoder;
//import edu.wpi.first.wpilibj.Spark;
//import edu.wpi.first.wpilibj.controller.PIDController;
//import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
//import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;

import frc.robot.Constants.ModuleConstants;


public class SwerveModule {
  private static final int TICKS = 16;
  public static final double kMaxAngularSpeed = Math.PI;
  private static final double kWheelRadius = 0.0508;
  private static final int kEncoderResolution = 4096;
  private static final double kModuleMaxAngularVelocity = kMaxAngularSpeed;
  private static final double kModuleMaxAngularAcceleration = 2 * Math.PI; // radians per second squared
  private final CANSparkMax m_driveMotor;
  private final CANSparkMax m_turningMotor;
  public CANEncoder m_driveEncoder;
  public CANEncoder m_turningEncoder;
  private CANPIDController m_drivePIDController;
  private CANPIDController m_turningPIDController;
    
  //private final Spark m_driveMotor;
  //private final Spark m_turningMotor;

  //private final Encoder m_driveEncoder;
  //private final Encoder m_turningEncoder;

  //private final PIDController m_drivePIDController =
  //    new PIDController(ModuleConstants.kPModuleDriveController, 0, 0);

  //Using a TrapezoidProfile PIDController to allow for smooth turning
  //private final ProfiledPIDController m_turningPIDController
  //    = new ProfiledPIDController(
  //        ModuleConstants.kPModuleTurningController, 0, 0,
  //        new TrapezoidProfile.Constraints(
  //            ModuleConstants.kMaxModuleAngularSpeedRadiansPerSecond,
  //            ModuleConstants.kMaxModuleAngularAccelerationRadiansPerSecondSquared));

  /**
   * Constructs a SwerveModule.
   *
   * @param driveMotorChannel   ID for the drive motor.
   * @param turningMotorChannel ID for the turning motor.
   */
  public SwerveModule(int driveMotorChannel, int turningMotorChannel) {
    int smartMotionSlot = 0;
    m_driveMotor = new CANSparkMax(driveMotorChannel, MotorType.kBrushless);
    m_driveMotor.restoreFactoryDefaults();
    m_drivePIDController = m_driveMotor.getPIDController();
    m_driveEncoder = m_driveMotor.getEncoder();
    m_drivePIDController.setP(5e-5);
    m_drivePIDController.setI(1e-6);
    m_drivePIDController.setD(0);
    m_drivePIDController.setFF(0.000156);
    m_drivePIDController.setOutputRange(-1, 1);
    m_drivePIDController.setSmartMotionMaxVelocity(2000, smartMotionSlot);
    m_drivePIDController.setSmartMotionMinOutputVelocity(0, smartMotionSlot);
    m_drivePIDController.setSmartMotionMaxAccel(1500, smartMotionSlot);
    m_drivePIDController.setSmartMotionAllowedClosedLoopError(50, smartMotionSlot);

    m_turningMotor = new CANSparkMax(turningMotorChannel, MotorType.kBrushless);
    m_turningMotor.restoreFactoryDefaults();
    m_turningPIDController = m_turningMotor.getPIDController();
    m_turningEncoder = m_turningMotor.getEncoder();
    m_turningMotor.setInverted(true);
    m_turningPIDController.setP(5e-5);
    m_turningPIDController.setI(1e-6);
    m_turningPIDController.setD(0);
    m_turningPIDController.setIZone(0);
    m_turningPIDController.setFF(0.000156);
    m_turningPIDController.setOutputRange(-1, 1);
    m_turningPIDController.setSmartMotionMaxVelocity(2000, smartMotionSlot);
    m_turningPIDController.setSmartMotionMinOutputVelocity(0, smartMotionSlot);
    m_turningPIDController.setSmartMotionMaxAccel(1500, smartMotionSlot);
    m_turningPIDController.setSmartMotionAllowedClosedLoopError(0, smartMotionSlot);

    //wheel = new Wheel(m_turningMotor, m_driveMotor, DriveSubsystem.DRIVE_SETPOINT_MAX);
    setAzimuthZero(0);

  //public SwerveModule(int driveMotorChannel,
  //                    int turningMotorChannel,
  //                    int[] driveEncoderPorts,
  //                    int[] turningEncoderPorts,
  //                    boolean driveEncoderReversed,
  //                    boolean turningEncoderReversed) {

  //  m_driveMotor = new Spark(driveMotorChannel);
  //  m_turningMotor = new Spark(turningMotorChannel);

  //  this.m_driveEncoder = new Encoder(driveEncoderPorts[0], driveEncoderPorts[1]);

  //  this.m_turningEncoder = new Encoder(turningEncoderPorts[0], turningEncoderPorts[1]);

    // Set the distance per pulse for the drive encoder. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.
    //m_driveEncoder.setDistancePerPulse(ModuleConstants.kDriveEncoderDistancePerPulse);

    //Set whether drive encoder should be reversed or not
    //m_driveEncoder.setReverseDirection(driveEncoderReversed);

    // Set the distance (in this case, angle) per pulse for the turning encoder.
    // This is the the angle through an entire rotation (2 * wpi::math::pi)
    // divided by the encoder resolution.
    //m_turningEncoder.setDistancePerPulse(ModuleConstants.kTurningEncoderDistancePerPulse);

    //Set whether turning encoder should be reversed or not
    //m_turningEncoder.setReverseDirection(turningEncoderReversed);

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    //m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
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
  private void setAzimuthZero(double zero) {
    //logger.info("<b>Wheel</b>: setAzimuthZero starting");
    double azimuthSetpoint = getAzimuthAbsolutePosition() - zero;
    //ErrorCode err = azimuthTalon.setSelectedSensorPosition(azimuthSetpoint, 0, 10);
    //Errors.check(err, logger);
    //azimuthTalon.set(MotionMagic, azimuthSetpoint);
    //azimuthSpark.set(azimuthSetpoint);
    //m_pidController.setReference(azimuthSetpoint, ControlType.kSmartMotion);
    m_turningEncoder.setPosition(0); // TODO change whith 221 encoder
    //logger.info("<b>Wheel</b>: setAzimuthZero finished");

  }

  public double getAzimuthAbsolutePosition() {
    //return azimuthTalon.getSensorCollection().getPulseWidthPosition() & 0xFFF;
    //return (int)azimuthSpark.get() & 0xFFF;
    //TODO - need to return azimuth from the 221 encoder
    return m_turningEncoder.getPosition(); //TODO FIXME RDB2020
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    //return new SwerveModuleState(m_driveEncoder.getRate(), new Rotation2d(m_turningEncoder.get()));
        //FIXME: apply any needed unit convertion here...
    double velocity = m_driveEncoder.getVelocity() * Math.PI * 3.0 / 39.37 / 60.0 / 16.0;
    double azimuth = m_turningEncoder.getPosition();
    double azimuthPercent = Math.IEEEremainder(azimuth, TICKS)/16.0;

    return new SwerveModuleState(velocity, new Rotation2d(azimuthPercent * 2.0 * Math.PI));

  }

  /**
   * Sets the desired state for the module.
   *
   * @param state Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState state) {
    // Calculate the drive output from the drive PID controller.
    //final var driveOutput = m_drivePIDController.calculate(
    //    m_driveEncoder.getRate(), state.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    //final var turnOutput = m_turningPIDController.calculate(
    //    m_turningEncoder.get(), state.angle.getRadians()
    //);

    // Calculate the turning motor output from the turning PID controller.
    //m_driveMotor.set(driveOutput);
    //m_turningMotor.set(turnOutput);
    
    double azimuth = -state.angle.getDegrees() * TICKS/360.0;
    double speedMetersPerSecond = state.speedMetersPerSecond;
    double drive = speedMetersPerSecond * 16.0 * 39.37  * 60.0 / 3.0 / Math.PI;
    //wheel.set(-angleDegrees/360, speedMetersPerSecond * 16.0 * 39.37  * 60.0 / 3.0 / Math.PI);
    double azimuthPosition = m_turningEncoder.getPosition();
    double azimuthError = Math.IEEEremainder(azimuth - azimuthPosition, TICKS);

    // minimize azimuth rotation, reversing drive if necessary
    boolean isInverted = Math.abs(azimuthError) > 0.25 * TICKS;
    if (isInverted) {
      azimuthError -= Math.copySign(0.5 * TICKS, azimuthError);
      drive = -drive;
    }
    m_turningPIDController.setReference((azimuthPosition + azimuthError), ControlType.kSmartMotion);

  }

  /**
   * Zeros all the SwerveModule encoders.
   */

  public void resetEncoders() {
    //m_driveEncoder.reset();
    //m_turningEncoder.reset();
    setAzimuthZero(0);
  }
}
