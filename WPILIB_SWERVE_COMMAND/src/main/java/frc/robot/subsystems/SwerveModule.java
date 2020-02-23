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

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static frc.robot.Constants.kTICKS;


public class SwerveModule {
  public static final double kMaxAngularSpeed = Math.PI;
  private CANSparkMax m_driveMotor;
  private CANSparkMax m_turningMotor;
  public CANEncoder m_driveEncoder;
  public CANEncoder m_turningEncoder;
  private CANPIDController m_drivePIDController;
  private CANPIDController m_turningPIDController;

  private double offsetFromAbsoluteEncoder;

  private int id;
    
  /**
   * Constructs a SwerveModule.
   *
   * @param driveMotorChannel   ID for the drive motor.
   * @param turningMotorChannel ID for the turning motor.
   */
  public SwerveModule(int driveMotorChannel, int turningMotorChannel) {
    id = driveMotorChannel;

    if(RobotBase.isReal()){
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
      m_drivePIDController.setSmartMotionMaxVelocity(2000, smartMotionSlot); //RPM
      m_drivePIDController.setSmartMotionMinOutputVelocity(0, smartMotionSlot);
      m_drivePIDController.setSmartMotionMaxAccel(1500, smartMotionSlot); //RPM per second
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
    }
    else{
      m_driveMotor = null;
      m_turningMotor = null;
    }

    //setAzimuthZero(0); //RDB 10FEB I don't think we want this any more -- abs encoders now
  }

public SwerveModule() {
  System.err.println("DUMMY SWERVE MODULE HAS BEEN INSTANTIATED");
}

public double getDriveEncoderPosition(){
  return m_driveEncoder.getPosition();
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
  private void setAzimuthZero(double zeroSetpointAbsoluteEncoderVoltage) { // 0.0 to 3.26, 180=1.63V
    offsetFromAbsoluteEncoder = zeroSetpointAbsoluteEncoderVoltage * 16/3.26;
    SmartDashboard.putNumber("offsetFromAbsoluteEncoder"+id, offsetFromAbsoluteEncoder);
    //logger.info("<b>Wheel</b>: setAzimuthZero starting");
    //double azimuthSetpoint = getAzimuthAbsolutePosition() - zero;
    //ErrorCode err = azimuthTalon.setSelectedSensorPosition(azimuthSetpoint, 0, 10);
    //Errors.check(err, logger);
    //azimuthTalon.set(MotionMagic, azimuthSetpoint);
    //azimuthSpark.set(azimuthSetpoint);
    //m_pidController.setReference(azimuthSetpoint, ControlType.kSmartMotion);
    //m_turningEncoder.setPosition(0); // TODO change whith 221 encoder
    //TODO FIXME change voltage to position 0 to 12!!
    //logger.info("<b>Wheel</b>: setAzimuthZero finished");
  }

  public double getAzimuthAbsolutePosition() {
    //return azimuthTalon.getSensorCollection().getPulseWidthPosition() & 0xFFF;
    //return (int)azimuthSpark.get() & 0xFFF;
    //TODO - need to return azimuth from the 221 encoder
    double rawEncoder = 0;
    if(RobotBase.isReal()){
      rawEncoder = m_turningEncoder.getPosition();
    }
    double correctedEncoder = rawEncoder - offsetFromAbsoluteEncoder;
    return correctedEncoder;
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    //return new SwerveModuleState(m_driveEncoder.getRate(), new Rotation2d(m_turningEncoder.get()));
        //FIXME: apply any needed unit convertion here...
    double velocity = 0;
    double azimuth = 0;
    if(RobotBase.isReal()){ // RPM/60 is RPS *PI*D is inches/s * 39.37 is meter/s but it's 5.5 ticks/rev
       velocity = (m_driveEncoder.getVelocity() * Math.PI * 3.0) / (39.37 * 60.0 * 5.5);
       azimuth = -m_turningEncoder.getPosition();
    }
    double azimuthPercent = Math.IEEEremainder(azimuth, kTICKS)/16.0;

    if(RobotBase.isReal()){
      SmartDashboard.putNumber("Module"+id+" Drive Encoder Tick", m_driveEncoder.getPosition());
    }

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
    
    double azimuth = -state.angle.getDegrees() * kTICKS/360.0;
    double speedMetersPerSecond = state.speedMetersPerSecond;
    SmartDashboard.putNumber("SpeedMPS-"+id, speedMetersPerSecond);
    // meters per sec * 39.37 is inches/s * 60 is inches per min / PI*D is RPM * 5.5 is ticks
    double drive = (speedMetersPerSecond * 5.5 * 39.37  * 60.0) / (3.0 * Math.PI);
    //wheel.set(-angleDegrees/360, speedMetersPerSecond * 16.0 * 39.37  * 60.0 / 3.0 / Math.PI);
    double azimuthPosition = 0;
    if(RobotBase.isReal()){
      azimuthPosition = m_turningEncoder.getPosition();
    }
    double azimuthError = Math.IEEEremainder(azimuth - azimuthPosition, kTICKS);

    // minimize azimuth rotation, reversing drive if necessary
    boolean isInverted = Math.abs(azimuthError) > 0.25 * kTICKS;
    if (isInverted) {
      azimuthError -= Math.copySign(0.5 * kTICKS, azimuthError);
      drive = -drive;
    }
    if(RobotBase.isReal()){
      m_turningPIDController.setReference((azimuthPosition + azimuthError), ControlType.kSmartMotion);
      m_drivePIDController.setReference(drive, ControlType.kVelocity);
    }

    SmartDashboard.putNumber("RelativeEncoder"+id, m_turningEncoder.getPosition());
    SmartDashboard.putNumber("absOffset"+id, offsetFromAbsoluteEncoder);
  }

  /**
   * Zeros all the SwerveModule encoders.
   */

  public void resetEncoders(double absoluteEncoderVoltage) {
    if(RobotBase.isReal()){
      m_driveEncoder.setPosition(0);
      m_turningEncoder.setPosition(absoluteEncoderVoltage * 16/3.26);
    }
  }

}
