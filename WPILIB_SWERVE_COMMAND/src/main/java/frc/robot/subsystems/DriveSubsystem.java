/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
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
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.TurnToAngleProfiled;
import frc.robot.util.DebugOutput;
import frc.robot.util.ReflectingCSVWriter;

@SuppressWarnings("PMD.ExcessiveImports")
public class DriveSubsystem extends SubsystemBase {

  private final ReflectingCSVWriter<DebugOutput> mCSVWriter;
  private final ReflectingCSVWriter<SwerveModuleDebugOutput> mCSVWriter2;
  private final DebugOutput debugOutput = new DebugOutput();
  private SwerveModuleDebugOutput swerveDebugOutput;
  private final ProfiledPIDController headingController = new ProfiledPIDController(
    DriveConstants.kTurnP, DriveConstants.kTurnI, DriveConstants.kTurnD,
    new TrapezoidProfile.Constraints(DriveConstants.kMaxTurnVelocity, DriveConstants.kMaxTurnAcceleration));

  //After looking inside the ProfiledPIDController class, I suspect that a standard PIDController will work better as ProfiledPID seems to primarily use the
  //trapezoid profiler to calculate the next output rather than the PID. Since trapezoid profiler doesn't have continuous input it just ignores it.
  //private final PIDController headingControllerPID = new PIDController(DriveConstants.kTurnP, DriveConstants.kTurnI, DriveConstants.kTurnD);
  
  //private double headingControllerOutput = 0;

  private boolean stickControlledHeading = true;
  
  private final AnalogInput leftFrontAbsEncoder;
  private final AnalogInput rightFrontAbsEncoder;
  private final AnalogInput leftRearAbsEncoder;
  private final AnalogInput rightRearAbsEncoder;
  
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
  

  private final SwerveModule m_rearRight =
      new SwerveModule(DriveConstants.kRearRightDriveMotorPort,
                       DriveConstants.kRearRightTurningMotorPort);

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

    leftFrontAbsEncoder = new AnalogInput(0);
    rightFrontAbsEncoder = new AnalogInput(1);
    leftRearAbsEncoder = new AnalogInput(2);
    rightRearAbsEncoder = new AnalogInput(3);

    if(RobotBase.isReal()){
      if(leftFrontAbsEncoder == null || rightFrontAbsEncoder == null || leftRearAbsEncoder == null || rightRearAbsEncoder == null){
        System.err.println("\n\nAt least one absolute encoder (AnalogInput(0)--AnalogInput(3) is NULL!!!\n\n");
      }
    }
    headingController.setTolerance(DriveConstants.kTurnToleranceDeg, DriveConstants.kTurnRateToleranceDegPerS);
    //headingController.enableContinuousInput(-180, 180);

    swerveDebugOutput = new SwerveModuleDebugOutput(m_frontLeft.getDebugValues(), m_frontRight.getDebugValues(), m_rearLeft.getDebugValues(), m_rearRight.getDebugValues());

    mCSVWriter = new ReflectingCSVWriter<>(this.getName(), DebugOutput.class);
    mCSVWriter2 = new ReflectingCSVWriter<>("SwerveModule", SwerveModuleDebugOutput.class);
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

  /*
  /**
   * @return the headingControllerOutput
  public double getHeadingControllerOutput() {
    return headingControllerOutput;
  }
  */

  @Override
  public void periodic() {
    resumeCSVWriter();

    //SmartDashboard.putNumber("headingController In", getHeading());
    //headingControllerOutput = headingController.calculate(MathUtil.clamp(getHeading(), -180, 180));
    //SmartDashboard.putNumber("headingController Out", headingControllerOutput);

    // Update the odometry in the periodic block
    double headingRadians = Math.toRadians(getHeading());
    m_odometry.update(
        new Rotation2d(headingRadians),
        m_frontLeft.getState(),              // frontLeft, frontRight, rearLeft, rearRight
        m_frontRight.getState(),
        m_rearLeft.getState(),
        m_rearRight.getState());
        
    if(System.currentTimeMillis() % 100 == 0){
      SmartDashboard.putNumber("pose x", m_odometry.getPoseMeters().getTranslation().getX());
      SmartDashboard.putNumber("pose y", m_odometry.getPoseMeters().getTranslation().getY());
      SmartDashboard.putNumber("rot deg", m_odometry.getPoseMeters().getRotation().getDegrees());
      SmartDashboard.putNumber("heading radians", headingRadians);    
      SmartDashboard.putNumber("raw gyro", m_gyro.getAngle());
      SmartDashboard.putBoolean("gyro is calibrating", m_gyro.isCalibrating());
    }
    debugOutput.update(Timer.getFPGATimestamp(), m_odometry, headingRadians, m_gyro.getAngle());
    swerveDebugOutput.update(m_frontLeft.getDebugValues(), m_frontRight.getDebugValues(), m_rearLeft.getDebugValues(), m_rearRight.getDebugValues());
    mCSVWriter.add(debugOutput);
    mCSVWriter2.add(swerveDebugOutput);
    
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
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative){
    drive(xSpeed, ySpeed, 0, 0, fieldRelative);
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
  public void drive(double xSpeed, double ySpeed, double rightX, double rightY, boolean fieldRelative) {
    double xSpeedAdjusted = xSpeed;
    double ySpeedAdjusted = ySpeed;
    double rotAdjusted = rightX;
    double rotationalOutput = 0;

    // DEADBAND
    if(Math.abs(xSpeedAdjusted) < 0.3){
      xSpeedAdjusted = 0;
    }
    if(Math.abs(ySpeedAdjusted) < 0.3){
      ySpeedAdjusted = 0;
    }
    if(Math.abs(rotAdjusted) < 0.3){
      rotAdjusted = 0;
    }

    /*
    if(stickControlledHeading){
      //If the stick is released, don't change the rotation
      if((Math.abs(rightX) > DriveConstants.kMinRightStickThreshold || Math.abs(rightY) > DriveConstants.kMinRightStickThreshold)){
        double stickAngle = getStickAngle(rightX, rightY);
        double heading = getHeading();

        //Continuous correction
        //This assumes that positive is COUNTERclockwise, which may be wrong... the fix should be simply reversing the signs
        if((stickAngle - heading) > 180){
          stickAngle += 360;
        } else if((stickAngle - heading) < -180){
          stickAngle -= 360;
        }

        rotationalOutput = headingController.calculate(heading, stickAngle);
      } else {
        headingController.reset(getHeading());
      }
    } else {
      //Perhaps do the same continuous correction here as above? Test first.
      rotationalOutput = headingController.calculate(getHeading());
    }
    */

    //Replaced rotAdjusted with rotationalOutput
    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
          xSpeedAdjusted, ySpeedAdjusted, rotAdjusted, getAngle())
            : new ChassisSpeeds(xSpeedAdjusted, ySpeedAdjusted, rotAdjusted)
    );
    SwerveDriveKinematics.normalizeWheelSpeeds(swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);    // frontLeft, frontRight, rearLeft, rearRight
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Turns the robot to a heading read from the gyro
   * @param heading The gyro angle from -180 to 180
   */
  public double getStickAngle(double stickX, double stickY){
      double stickAngle = Math.toDegrees(Math.atan2(stickY, stickX));
      stickAngle -= 90;
      
      //SmartDashboard.putNumber("getStickAngle Raw", stickAngle);

      //Don't know of Math.atan2 does this automatically. Check smartdashboard
      /*
      if((stickX < 0 && stickY >= 0) || (stickX < 0 && stickY < 0)){
        stickAngle += 180;
      } else if(stickX > 0 && stickY < 0){
        stickAngle += 360;
      }
      */

      //If the angle is greater than 180, mirror and invert it to keep the -180-180 heading angle (see getHeading())
      if(stickAngle < -180){
            stickAngle += 360;
      }
      
      stickAngle *= -1;

      //SmartDashboard.putNumber("getStickAngle Clamped", stickAngle);

      /*

      Currently, we need to find the quickest turn from -179 to 179 in heading. So in reality, 
      this is only a 2 degree difference
      However, the robot is spinning all the way around...



      */

      return stickAngle;

      //JavaScript from a cartesian to polar coordinate converter:
      //Tbh drives me mad just a little with the lack of an OR and the fact that there's an if statement that just does nothing
      /*
      var x = $j('#x').val();
 	var y = $j('#y').val();
 	var r = Math.pow((Math.pow(x,2) + Math.pow(y,2)),0.5);
 	var theta = Math.atan(y/x)*360/2/Math.PI;
 	if (x >= 0 && y >= 0) {
 		theta = theta;
 	} else if (x < 0 && y >= 0) {
 		theta = 180 + theta;
 	} else if (x < 0 && y < 0) {
 		theta = 180 + theta;
 	} else if (x > 0 && y < 0) {
 		theta = 360 + theta;
   } 
   */
      
      
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
  public void resetEncoders() {
    m_frontLeft.resetEncoders(leftFrontAbsEncoder.getVoltage());     // frontLeft, frontRight, rearLeft, rearRight
    //m_frontRight.resetEncoders(rightFrontAbsEncoder.getVoltage()); took this one out -- bad hardware encoder!!!
    m_rearLeft.resetEncoders(leftRearAbsEncoder.getVoltage());
    m_rearRight.resetEncoders(rightRearAbsEncoder.getVoltage());
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
    if(System.currentTimeMillis() % 100 == 0){  
      SmartDashboard.putNumber("Heading", heading);
    }
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

    if(!mCSVWriter2.isSuspended()){
      mCSVWriter2.suspend();
    }
  }

  public void resumeCSVWriter() {
    if(mCSVWriter.isSuspended()){
      mCSVWriter.resume();
    }

    if(mCSVWriter2.isSuspended()){
      mCSVWriter2.resume();
    }
  }

  /**
   * Determines if the right thumbstick on the driver controller can control the robot's orientation.
   * Set to false if you want another subsystem to control the orientation in teleop (e.x. vision)
   */
  public void setStickControlledHeading(boolean controlledHeading){
    stickControlledHeading = controlledHeading;
  }

  public void setHeadingControllerGoal(double newGoal){
    headingController.setGoal(newGoal);
  }


  public void displayEncoders() {
    //SmartDashboard.putNumber("leftFrontAbsEncoder", leftFrontAbsEncoder.getVoltage()); // 0.0 to 3.26, 180=1.63V
    //SmartDashboard.putNumber("rightFrontAbsEncoder", rightFrontAbsEncoder.getVoltage()); // 0.0 to 3.26, 180=1.63V
    //SmartDashboard.putNumber("leftRearAbsEncoder", leftRearAbsEncoder.getVoltage()); // 0.0 to 3.26, 180=1.63V
    //SmartDashboard.putNumber("rightRearAbsEncoder", rightRearAbsEncoder.getVoltage()); // 0.0 to 3.26, 180=1.63V
  }


  public void resetGyro() {
    if(m_gyro != null){
      m_gyro.reset();
    }
  }

  public class SwerveModuleDebugOutput {
    public double drive1AppliedOutput;
    public double drive2AppliedOutput;
    public double drive3AppliedOutput;
    public double drive4AppliedOutput;

    public double drive1Velocity;
    public double drive2Velocity;
    public double drive3Velocity;
    public double drive4Velocity;

    public double turn1AppliedOutput;
    public double turn2AppliedOutput;
    public double turn3AppliedOutput;
    public double turn4AppliedOutput;

    public double turn1Velocity;
    public double turn2Velocity;
    public double turn3Velocity;
    public double turn4Velocity;

    public SwerveModuleDebugOutput(SwerveModule.DebugValues debugValues1, SwerveModule.DebugValues debugValues2, 
    SwerveModule.DebugValues debugValues3, SwerveModule.DebugValues debugValues4){
      update(debugValues1, debugValues2, debugValues3, debugValues4);
    }

    public void update(SwerveModule.DebugValues debugValues1, SwerveModule.DebugValues debugValues2, 
    SwerveModule.DebugValues debugValues3, SwerveModule.DebugValues debugValues4){
      drive1AppliedOutput = debugValues1.driveAppliedOutput;
      drive1Velocity = debugValues1.driveVelocity;
      turn1AppliedOutput = debugValues1.turnAppliedOutput;
      turn1Velocity = debugValues1.turnVelocity;

      drive2AppliedOutput = debugValues2.driveAppliedOutput;
      drive2Velocity = debugValues2.driveVelocity;
      turn2AppliedOutput = debugValues2.turnAppliedOutput;
      turn2Velocity = debugValues2.turnVelocity;

      drive3AppliedOutput = debugValues3.driveAppliedOutput;
      drive3Velocity = debugValues3.driveVelocity;
      turn3AppliedOutput = debugValues3.turnAppliedOutput;
      turn3Velocity = debugValues3.turnVelocity;

      drive4AppliedOutput = debugValues4.driveAppliedOutput;
      drive4Velocity = debugValues4.driveVelocity;
      turn4AppliedOutput = debugValues4.turnAppliedOutput;
      turn4Velocity = debugValues4.turnVelocity;
    }

  }
}
