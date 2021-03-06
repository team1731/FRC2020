/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package swervebot;

import com.kauailabs.navx.frc.AHRS;

import org.strykeforce.thirdcoast.swerve.SwerveDrive;
import org.usfirst.frc.team1731.lib.util.drivers.NavX;
import org.usfirst.frc.team1731.robot.Constants;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Represents a swerve drive style drivetrain.
 */
public class Drivetrain {

  public static final double kMaxSpeed = 1.0; // 3 meters per second
  public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second

  private final Translation2d m_frontLeftLocation = new Translation2d(0.381, 0.381);
  private final Translation2d m_frontRightLocation = new Translation2d(0.381, -0.381);
  private final Translation2d m_backLeftLocation = new Translation2d(-0.381, 0.381);
  private final Translation2d m_backRightLocation = new Translation2d(-0.381, -0.381);

  private final SwerveModule m_frontLeft = new SwerveModule(1, 11);
  private final SwerveModule m_frontRight = new SwerveModule(2, 12);
  private final SwerveModule m_backLeft = new SwerveModule(3, 13);
  private final SwerveModule m_backRight = new SwerveModule(4, 14);

  //private final AnalogGyro m_gyro = new AnalogGyro(0);
  AHRS m_gyro  = NavX.getAHRS();

  public void zeroGyro() {
    //logger.info("<b>DriveSubsystem</b>: zeroGyro started");
    m_gyro.setAngleAdjustment(0);
    double adj = m_gyro.getAngle() % 360;
    m_gyro.setAngleAdjustment(-adj);
    //logger.info("<b>DriveSubsystem</b>: zeroGyro finished");
  }

  // Swerve configuration
  private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
      m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation
  );

  private final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(m_kinematics, getAngle());

  public synchronized SwerveDriveOdometry getWPILibOdometry(){
      return m_odometry;
  }

  public Drivetrain() {
    m_gyro.reset();
  }

  /**
   * Returns the angle of the robot as a Rotation2d.
   *
   * @return The angle of the robot.
   */
  public Rotation2d getAngle() {
    // Negating the angle because WPILib gyros are CW positive.
    return Rotation2d.fromDegrees(-m_gyro.getAngle());
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
    SwerveModuleState[] swerveModuleStates = m_kinematics.toSwerveModuleStates(
        fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
            xSpeed, ySpeed, rot, getAngle())
            : new ChassisSpeeds(xSpeed, ySpeed, rot)
    );
    SwerveDriveKinematics.normalizeWheelSpeeds(swerveModuleStates, kMaxSpeed);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Updates the field relative position of the robot.
   */
  public void updateOdometry() {
    m_odometry.update(
        getAngle(),
        m_frontLeft.getState(),
        m_frontRight.getState(),
        m_backLeft.getState(),
        m_backRight.getState()
    );
    SmartDashboard.putNumber("pose x", m_odometry.getPoseMeters().getTranslation().getX() * Constants.INCHES_PER_METER);
    SmartDashboard.putNumber("pose y", m_odometry.getPoseMeters().getTranslation().getY() * Constants.INCHES_PER_METER);
    SmartDashboard.putNumber("rot", m_odometry.getPoseMeters().getRotation().getDegrees());
    SmartDashboard.putNumber("raw gyro", m_gyro.getAngle());

  }
}
