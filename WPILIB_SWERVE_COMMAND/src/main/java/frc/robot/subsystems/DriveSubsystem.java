/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.AnalogInput;
//import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
//import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.DriveConstants;
import frc.robot.util.DebugOutput;
import frc.robot.util.ReflectingCSVWriter;

@SuppressWarnings("PMD.ExcessiveImports")
public class DriveSubsystem extends SubsystemBase {

  private final ReflectingCSVWriter<DebugOutput> mCSVWriter;
  private final DebugOutput debugOutput = new DebugOutput();
  
  //Robot swerve modules
  private final SwerveModule m_frontLeft = 
      new SwerveModule(DriveConstants.kFrontLeftDriveMotorPort,
                        DriveConstants.kFrontLeftTurningMotorPort);

  private final SwerveModule m_frontRight =
      new SwerveModule(DriveConstants.kFrontRightDriveMotorPort,
                       DriveConstants.kFrontRightTurningMotorPort);

  private final SwerveModule m_rearLeft =
      new SwerveModule(DriveConstants.kRearLeftDriveMotorPort,
                       DriveConstants.kRearLeftTurningMotorPort);
  

  private final SwerveModule m_rearRight = null;
      // new SwerveModule(DriveConstants.kRearRightDriveMotorPort,
      //                  DriveConstants.kRearRightTurningMotorPort);

  // The gyro sensor
  //private final Gyro a_gyro = new ADXRS450_Gyro();
  private final AHRS m_gyro = new AHRS(SPI.Port.kMXP);

  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry =
      new SwerveDriveOdometry(DriveConstants.kDriveKinematics, getAngle());

  /**
   * Creates a new DriveSubsystem.
   */
  public DriveSubsystem() {
    mCSVWriter = new ReflectingCSVWriter<>(this.getName(), DebugOutput.class);
  }

  
  /**
   * Returns the angle of the robot as a Rotation2d.
   *
   * @return The angle of the robot.
   */
  public Rotation2d getAngle() {
    // Negating the angle because WPILib gyros are CW positive.
    return Rotation2d.fromDegrees(m_gyro.getAngle() * (DriveConstants.kGyroReversed ? -1.0 : 1.0));
  }

  @Override
  public void periodic() {
    if(mCSVWriter.isSuspended()){
      mCSVWriter.resume();
    }

    // Update the odometry in the periodic block
    double headingRadians = Math.toRadians(getHeading());
    m_odometry.update(
        new Rotation2d(headingRadians),
        m_frontLeft.getState(),              // frontLeft, frontRight, rearLeft, rearRight
        m_frontRight.getState(),
        m_rearLeft.getState(),
        new SwerveModuleState()); //m_rearRight.getState());
    SmartDashboard.putNumber("pose x", m_odometry.getPoseMeters().getTranslation().getX());
    SmartDashboard.putNumber("pose y", m_odometry.getPoseMeters().getTranslation().getY());
    SmartDashboard.putNumber("rot deg", m_odometry.getPoseMeters().getRotation().getDegrees());
    SmartDashboard.putNumber("heading radians", headingRadians);    
    SmartDashboard.putNumber("raw gyro", m_gyro.getAngle());
    SmartDashboard.putBoolean("gyro is calibrating", m_gyro.isCalibrating());
    debugOutput.update(Timer.getFPGATimestamp(), m_odometry, headingRadians, m_gyro.getAngle());
    mCSVWriter.add(debugOutput);
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(pose, getAngle());
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  @SuppressWarnings("ParameterName")
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    double xSpeedAdjusted = xSpeed;
    double ySpeedAdjusted = ySpeed;
    double rotAdjusted = rot;
    // DEADBAND
    if(Math.abs(xSpeedAdjusted) < 0.05){
      xSpeedAdjusted = 0;
    }
    if(Math.abs(ySpeedAdjusted) < 0.05){
      ySpeedAdjusted = 0;
    }
    if(Math.abs(rotAdjusted) < 0.05){
      rotAdjusted = 0;
    }
    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
          xSpeedAdjusted, ySpeedAdjusted, rotAdjusted, getAngle())
            : new ChassisSpeeds(xSpeedAdjusted, ySpeedAdjusted, rotAdjusted)
    );
    SwerveDriveKinematics.normalizeWheelSpeeds(swerveModuleStates,
                                               DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);    // frontLeft, frontRight, rearLeft, rearRight
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.normalizeWheelSpeeds(desiredStates,
                                               DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);           // frontLeft, frontRight, rearLeft, rearRight
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /**
   * Resets the drive encoders to currently read a position of 0.
   */
  public void resetEncoders(double leftFrontAbsEncoderReading, // frontLeft, frontRight, rearLeft, rearRight
                            double rightFrontAbsEncoderReading, 
                            double leftRearAbsEncoderReading, 
                            double rightRearAbsEncoderReading) {
    m_frontLeft.resetEncoders(leftFrontAbsEncoderReading);     // frontLeft, frontRight, rearLeft, rearRight
    m_frontRight.resetEncoders(rightFrontAbsEncoderReading);
    m_rearLeft.resetEncoders(leftRearAbsEncoderReading);
    m_rearRight.resetEncoders(rightRearAbsEncoderReading);
  }

  /**
   * Zeroes the heading of the robot.
   */
  public void zeroHeading() {
    m_gyro.reset(); // RDB2020 - I replace this call with the below 5 lines...

    //logger.info("<b>DriveSubsystem</b>: zeroGyro started");
    //m_gyro.setAngleAdjustment(0);
    //double adj = m_gyro.getAngle() % 360;
    //m_gyro.setAngleAdjustment(-adj);
    //logger.info("<b>DriveSubsystem</b>: zeroGyro finished");
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    double heading = Math.IEEEremainder(m_gyro.getAngle(), 360) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
    SmartDashboard.putNumber("Heading", heading);
    return heading;
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  public void suspendCSVWriter() {
    if(!mCSVWriter.isSuspended()){
      mCSVWriter.suspend();
    }
  }

  public void resumeCSVWriter() {
    if(mCSVWriter.isSuspended()){
      mCSVWriter.resume();
    }
  }
}
