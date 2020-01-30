/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package swervebot;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import org.strykeforce.thirdcoast.swerve.Wheel;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import frc.robot.subsystem.DriveSubsystem;

public class SwerveModule {
  private static final double kWheelRadius = 0.0508;
  private static final int kEncoderResolution = 4096;

  private static final double kModuleMaxAngularVelocity = Drivetrain.kMaxAngularSpeed;
  private static final double kModuleMaxAngularAcceleration = 2 * Math.PI; // radians per second squared

  //private final SpeedController m_driveMotor;
  //private final SpeedController m_turningMotor;
  private final CANSparkMax m_driveMotor;
  private final CANSparkMax m_turningMotor;

  //private final Encoder m_driveEncoder = new Encoder(0, 1);
  //private final Encoder m_turningEncoder = new Encoder(2, 3);
  public CANEncoder m_driveEncoder;
  public CANEncoder m_turningEncoder;

  //private final PIDController m_drivePIDController = new PIDController(1, 0, 0);
  private CANPIDController m_drivePIDController;
  
  //private final ProfiledPIDController m_turningPIDController = new ProfiledPIDController(1, 0, 0,
  //        new TrapezoidProfile.Constraints(kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration));
  private CANPIDController m_turningPIDController;

  // Gains are for example purposes only - must be determined for your own robot!
  //private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(1, 3);
  //private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(1, 0.5);

  private final Wheel wheel;
  
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

    // Set the distance per pulse for the drive encoder. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.
    //m_driveEncoder.setDistancePerPulse(2 * Math.PI * kWheelRadius / kEncoderResolution);

    // Set the distance (in this case, angle) per pulse for the turning encoder.
    // This is the the angle through an entire rotation (2 * wpi::math::pi)
    // divided by the encoder resolution.
    //m_turningEncoder.setDistancePerPulse(2 * Math.PI / kEncoderResolution);

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    //m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);

    wheel = new Wheel(m_driveMotor, m_turningMotor, DriveSubsystem.DRIVE_SETPOINT_MAX);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    //return new SwerveModuleState(m_driveEncoder.getRate(), new Rotation2d(m_turningEncoder.get()));
    return new SwerveModuleState(m_driveEncoder.getVelocity(), new Rotation2d(m_turningEncoder.getPosition()));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param state Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState state) {
    // Calculate the drive output from the drive PID controller.
    //final double driveOutput = m_drivePIDController.calculate(m_driveEncoder.getRate(), state.speedMetersPerSecond);
    //final double driveFeedforward = m_driveFeedforward.calculate(state.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    //final double turnOutput = m_turningPIDController.calculate(m_turningEncoder.get(), state.angle.getRadians());
    //final double turnFeedforward = m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);

    //m_driveMotor.setVoltage(driveOutput + driveFeedforward);
    //m_turningMotor.setVoltage(turnOutput + turnFeedforward);

    wheel.set(state.angle.getDegrees(), state.speedMetersPerSecond/39.37); //FIXME: is this supposed to be inches per second???
  }
}
