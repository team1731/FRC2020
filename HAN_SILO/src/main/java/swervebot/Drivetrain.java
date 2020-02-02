/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package swervebot;

import java.util.Optional;

import com.kauailabs.navx.frc.AHRS;

import org.strykeforce.thirdcoast.swerve.SwerveDrive;
import org.usfirst.frc.team1731.lib.util.DriveSignal;
import org.usfirst.frc.team1731.lib.util.ReflectingCSVWriter;
import org.usfirst.frc.team1731.lib.util.control.Lookahead;
import org.usfirst.frc.team1731.lib.util.control.Path;
import org.usfirst.frc.team1731.lib.util.control.PathFollower;
import org.usfirst.frc.team1731.lib.util.control.PathFollower.DebugOutput;
import org.usfirst.frc.team1731.lib.util.drivers.NavX;
import org.usfirst.frc.team1731.lib.util.math.RigidTransform2d;
import org.usfirst.frc.team1731.lib.util.math.Twist2d;
import org.usfirst.frc.team1731.robot.Constants;
import org.usfirst.frc.team1731.robot.Kinematics;
import org.usfirst.frc.team1731.robot.RobotState;
import org.usfirst.frc.team1731.robot.ShooterAimingParameters;
import org.usfirst.frc.team1731.robot.loops.Loop;

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
public class Drivetrain implements Loop {

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

  private boolean mIsOnTarget = false;
  private org.usfirst.frc.team1731.lib.util.math.Rotation2d mTargetHeading = new org.usfirst.frc.team1731.lib.util.math.Rotation2d();

  private RobotState mRobotState = RobotState.getInstance();
  private final ReflectingCSVWriter<PathFollower.DebugOutput> mCSVWriter;
  
  // The robot drivetrain's various states.
  public enum DriveControlState {
      OPEN_LOOP, // open loop voltage control
      VELOCITY_SETPOINT, // velocity PID control
      PATH_FOLLOWING, // used for autonomous driving
      AIM_TO_GOAL, // turn to face the boiler
      TURN_TO_HEADING, // turn in place
      DRIVE_TOWARDS_GOAL_COARSE_ALIGN, // turn to face the boiler, then DRIVE_TOWARDS_GOAL_COARSE_ALIGN
      DRIVE_TOWARDS_GOAL_APPROACH, // drive forwards until we are at optimal shooting distance
  }

  @Override
  public void onStart(double timestamp) {
      synchronized (Drivetrain.this) {
          //setOpenLoop(DriveSignal.NEUTRAL);
          setBrakeMode(false);
          setVelocitySetpoint(0, 0);
          //mNavXBoard.reset();
      }
  }

  @Override
  public void onLoop(double timestamp) {
      synchronized (Drivetrain.this) {
          switch (mDriveControlState) {
          case OPEN_LOOP:
              return;
          case VELOCITY_SETPOINT:
              return;
          case PATH_FOLLOWING:
              if (mPathFollower != null) {
                  updatePathFollower(timestamp);
                  DebugOutput output = mPathFollower.getDebug();
                  mCSVWriter.add(output);
                  System.out.println(output.pose_x+", "+output.pose_y);
              }
              return;
          case AIM_TO_GOAL:
//                   if (!Superstructure.getInstance().isShooting()) {
                  updateGoalHeading(timestamp);
//                   }
              // fallthrough intended
          case TURN_TO_HEADING:
              updateTurnToHeading(timestamp);
              return;
          case DRIVE_TOWARDS_GOAL_COARSE_ALIGN:
              updateDriveTowardsGoalCoarseAlign(timestamp);
              return;
          case DRIVE_TOWARDS_GOAL_APPROACH:
              updateDriveTowardsGoalApproach(timestamp);
              return;
          default:
              System.out.println("Unexpected drive control state: " + mDriveControlState);
              break;
          }
      }
  }

  @Override
  public void onStop(double timestamp) {
      stop();
      mCSVWriter.flush();
      mCSVWriter.write();
      //mCSVWriter2.flush();
  }

    public synchronized void stop() {
        return;
    }

    private void setBrakeMode(boolean on){
        m_frontLeft.setBrakeMode(on);
        m_frontRight.setBrakeMode(on);
        m_backLeft.setBrakeMode(on);
        m_backRight.setBrakeMode(on);
    }
  /**
     * Check if the drive talons are configured for velocity control
     */
    protected static boolean usesTalonVelocityControl(DriveControlState state) {
      if (state == DriveControlState.VELOCITY_SETPOINT || state == DriveControlState.PATH_FOLLOWING) {
          return true;
      }
      return false;
  }

  /**
   * Check if the drive talons are configured for position control
   */
  protected static boolean usesTalonPositionControl(DriveControlState state) {
      if (state == DriveControlState.AIM_TO_GOAL ||
              state == DriveControlState.TURN_TO_HEADING ||
              state == DriveControlState.DRIVE_TOWARDS_GOAL_COARSE_ALIGN ||
              state == DriveControlState.DRIVE_TOWARDS_GOAL_APPROACH) {
          return true;
      }
      return false;
  }

  // Control states
  private DriveControlState mDriveControlState;

  private Path mCurrentPath = null;
  private PathFollower mPathFollower;

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
    mCSVWriter = new ReflectingCSVWriter<PathFollower.DebugOutput>("/home/lvuser/PATH-FOLLOWER-LOGS.csv",
                PathFollower.DebugOutput.class);
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

  public synchronized boolean isDoneWithPath() {
      if (mDriveControlState == DriveControlState.PATH_FOLLOWING && mPathFollower != null) {
          return mPathFollower.isFinished();
      } else {
        // System.out.println("Robot is not in path following mode");
          return true;
      }
  }



    public synchronized void forceDoneWithPath() {
        if (mDriveControlState == DriveControlState.PATH_FOLLOWING && mPathFollower != null) {
            mPathFollower.forceFinish();
        } else {
            //System.out.println("Robot is not in path following mode");
        }
    }

    /**
     * Configures the drivebase to turn to a desired heading
     */
    public synchronized void setWantTurnToHeading(org.usfirst.frc.team1731.lib.util.math.Rotation2d heading) {
      System.err.println("NEED A NEW METHOD FOR setWantTurnToHeading");
      if (mDriveControlState != DriveControlState.TURN_TO_HEADING) {
          //configureTalonsForPositionControl();
          mDriveControlState = DriveControlState.TURN_TO_HEADING;
          //updatePositionSetpoint(getLeftDistanceInches(), getRightDistanceInches());
      }
   //   if (Math.abs(heading.inverse().rotateBy(mTargetHeading).getDegrees()) > 1E-3) {
          mTargetHeading = heading;
          mIsOnTarget = false;
   //   }
   //   setHighGear(false);
  }

  private void updateDriveTowardsGoalApproach(double timestamp){
    return;
  }

  private void updateDriveTowardsGoalCoarseAlign(double timestamp){
    return;
  }

  /**
     * Turn the robot to a target heading.
     * 
     * Is called periodically when the robot is auto-aiming towards the boiler.
     */
    private void updateTurnToHeading(double timestamp) {
      return;
      /*
      final org.usfirst.frc.team1731.lib.util.math.Rotation2d field_to_robot = mRobotState.getLatestFieldToVehicle().getRotation();

      // Figure out the rotation necessary to turn to face the goal.
      final org.usfirst.frc.team1731.lib.util.math.Rotation2d robot_to_target = field_to_robot.inverse().rotateBy(mTargetHeading);

      // Check if we are on target
      final double kGoalPosTolerance = 3.0; // degrees  254 had .75
      final double kGoalVelTolerance = 5.0; // inches per second
      if (Math.abs(robot_to_target.getDegrees()) < kGoalPosTolerance
              && Math.abs(getLeftVelocityInchesPerSec()) < kGoalVelTolerance
              && Math.abs(getRightVelocityInchesPerSec()) < kGoalVelTolerance) {
          // Use the current setpoint and base lock.
          mIsOnTarget = true;
          updatePositionSetpoint(getLeftDistanceInches(), getRightDistanceInches());
          return;
      }

      Kinematics.DriveVelocity wheel_delta = Kinematics
              .inverseKinematics(new Twist2d(0, 0, robot_to_target.getRadians()));
      updatePositionSetpoint(wheel_delta.left + getLeftDistanceInches(),
              wheel_delta.right + getRightDistanceInches());
              */
  }

    /**
     * Update the heading at which the robot thinks the boiler is.
     * 
     * Is called periodically when the robot is auto-aiming towards the boiler.
     */
    private void updateGoalHeading(double timestamp) {
        Optional<ShooterAimingParameters> aim = mRobotState.getAimingParameters();
        if (aim.isPresent()) {
            mTargetHeading = aim.get().getRobotToGoal();
            
        }
    }

  /**
     * Called periodically when the robot is in path following mode. Updates the path follower with the robots latest
     * pose, distance driven, and velocity, the updates the wheel velocity setpoints.
     */
    private void updatePathFollower(double timestamp) {
      RigidTransform2d robot_pose = mRobotState.getLatestFieldToVehicle();
      Twist2d command = mPathFollower.update(timestamp, robot_pose,
              mRobotState.getDistanceDriven(), RobotState.getInstance().getPredictedVelocity().dx);
      if (!mPathFollower.isFinished()) {
          //Kinematics.DriveVelocity setpoint = Kinematics.inverseKinematics(command);
          //updateVelocitySetpoint(setpoint.left, setpoint.right);

          updateVelocitySetpoint(command.dx, command.dtheta);

          //org.usfirst.frc.team1731.lib.util.math.Translation2d poseTrans = robot_pose.getTranslation();
          //Path.TargetPointReport report = mCurrentPath.getTargetPoint(poseTrans, new Lookahead());
      } else {
          updateVelocitySetpoint(0, 0);
      }
  }
  
  /**
     * Configures the drivebase to drive a path. Used for autonomous driving
     * 
     * @see Path
     */
    public synchronized void setWantDrivePath(Path path, boolean reversed) {
      if (mCurrentPath != path || mDriveControlState != DriveControlState.PATH_FOLLOWING) {
          //configureTalonsForSpeedControl();
          RobotState.getInstance().resetDistanceDriven();
          mPathFollower = new PathFollower(path, reversed,
                  new PathFollower.Parameters(
                          new Lookahead(),
                          Constants.kInertiaSteeringGain, Constants.kPathFollowingProfileKp,
                          Constants.kPathFollowingProfileKi, Constants.kPathFollowingProfileKv,
                          Constants.kPathFollowingProfileKffv, Constants.kPathFollowingProfileKffa,
                          Constants.kPathFollowingMaxVel, Constants.kPathFollowingMaxAccel,
                          Constants.kPathFollowingGoalPosTolerance, Constants.kPathFollowingGoalVelTolerance,
                          Constants.kPathStopSteeringDistance));
          mDriveControlState = DriveControlState.PATH_FOLLOWING;
          mCurrentPath = path;
      } else {
          setVelocitySetpoint(0, 0);
      }
  }

  /**
     * Start up velocity mode. This sets the drive train in high gear as well.
     * 
     * @param left_inches_per_sec
     * @param right_inches_per_sec
     */
    public synchronized void setVelocitySetpoint(double left_inches_per_sec, double right_inches_per_sec) {
        mDriveControlState = DriveControlState.VELOCITY_SETPOINT;
        updateVelocitySetpoint(left_inches_per_sec, right_inches_per_sec);
    }

    /**
     * Adjust Velocity setpoint (if already in velocity mode)
     * 
     * @param left_inches_per_sec
     * @param right_inches_per_sec
     */
    private synchronized void updateVelocitySetpoint(double dx, double dtheta) {
      if (usesTalonVelocityControl(mDriveControlState)) {
          //final double max_desired = Math.max(Math.abs(left_inches_per_sec), Math.abs(right_inches_per_sec));
          //final double scale = max_desired > Constants.kDriveHighGearMaxSetpoint
          //        ? Constants.kDriveHighGearMaxSetpoint / max_desired : 1.0;
            //mLeftMaster.set(ControlMode.Velocity, inchesPerSecondToUnitsPer100ms(left_inches_per_sec * scale));
            //mRightMaster.set(ControlMode.Velocity, inchesPerSecondToUnitsPer100ms(right_inches_per_sec * scale));

            double y = dx * Math.sin(Math.toRadians(dtheta));
            double x = dx * Math.cos(Math.toRadians(dtheta));

            drive(x, y, 0, true);
      } else {
          System.out.println("Hit a bad velocity control state");
          //mLeftMaster.set(ControlMode.Velocity,0);
          //mRightMaster.set(ControlMode.Velocity,0);
      }
    }

    private static double rotationsToInches(double rotations) {
      return rotations * (Constants.kDriveWheelDiameterInches * Math.PI);
    }

    private static double rpmToInchesPerSecond(double rpm) {
        return rotationsToInches(rpm) / 60;
    }

    private static double inchesToRotations(double inches) {
        return inches / (Constants.kDriveWheelDiameterInches * Math.PI);
    }

    private static double inchesPerSecondToRpm(double inches_per_second) {
        return inchesToRotations(inches_per_second) * 60;
    }
    
    private static double inchesPerSecondToUnitsPer100ms(double inches_per_second) {
      return ((inchesPerSecondToRpm(inches_per_second))*(4096.0)/600.0);
    }
}
