package org.strykeforce.thirdcoast.swerve;

import java.util.Optional;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.thirdcoast.talon.Errors;
import org.usfirst.frc.team1731.lib.util.DriveSignal;
import org.usfirst.frc.team1731.lib.util.ReflectingCSVWriter;
import org.usfirst.frc.team1731.lib.util.Util;
import org.usfirst.frc.team1731.lib.util.control.Lookahead;
import org.usfirst.frc.team1731.lib.util.control.Path;
import org.usfirst.frc.team1731.lib.util.control.PathFollower;
import org.usfirst.frc.team1731.lib.util.drivers.NavX;
import org.usfirst.frc.team1731.lib.util.math.RigidTransform2d;
import org.usfirst.frc.team1731.lib.util.math.Rotation2d;
import org.usfirst.frc.team1731.lib.util.math.Twist2d;
import org.usfirst.frc.team1731.lib.util.motion.SetpointGenerator.Setpoint;
import org.usfirst.frc.team1731.robot.Constants;
import org.usfirst.frc.team1731.robot.Kinematics;
import org.usfirst.frc.team1731.robot.RobotState;
import org.usfirst.frc.team1731.robot.ShooterAimingParameters;
import org.usfirst.frc.team1731.robot.loops.Loop;
import org.usfirst.frc.team1731.robot.loops.Looper;
import org.usfirst.frc.team1731.robot.subsystems.Subsystem;
import org.usfirst.frc.team1731.robot.subsystems.Drive.DriveControlState;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Robot;
import frc.robot.control.DriverControls;
import frc.robot.subsystem.DriveSubsystem;

/**
 * Control a Third Coast swerve drive.
 *
 * <p>
 * Wheels are a array numbered 0-3 from front to back, with even numbers on the
 * left side when facing forward.
 *
 * <p>
 * Derivation of inverse kinematic equations are from Ether's
 * <a href="https://www.chiefdelphi.com/media/papers/2426">Swerve Kinematics and
 * Programming</a>.
 *
 * @see Wheel
 */
@SuppressWarnings("unused")
public class SwerveDrive extends Subsystem {

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

  private final ReflectingCSVWriter<PathFollower.DebugOutput> mCSVWriter = new ReflectingCSVWriter<PathFollower.DebugOutput>("/home/lvuser/PATH-FOLLOWER-LOGS.csv", PathFollower.DebugOutput.class);

  private RobotState mRobotState = RobotState.getInstance();
  private Path mCurrentPath = null;
  private PathFollower mPathFollower;
  private final NavX mNavXBoard;

  private boolean mIsOnTarget = false;
  private boolean mIsApproaching = false;
  private Rotation2d mTargetHeading = new Rotation2d();

  private DriveControlState mDriveControlState;

  private Loop mLoop = new Loop(){

    @Override
    public void onStart(double timestamp) {
      synchronized(SwerveDrive.this){
        setOpenLoop(DriveSignal.NEUTRAL);
        //setBrakeMode(false);
        setVelocitySetpoint(0, 0);
        //mNavXBoard.reset();
      }
    }

    @Override
    public void onLoop(double timestamp) {
      synchronized (SwerveDrive.this) {
        switch (mDriveControlState) {
        case OPEN_LOOP:
            return;
        case CLIMB_BACKUP:
            return;
        case VELOCITY_SETPOINT:
            return;
        case PATH_FOLLOWING:
            if (mPathFollower != null) {
                updatePathFollower(timestamp);
                mCSVWriter.add(mPathFollower.getDebug());
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
            /*
        case TRACTOR_BEAM:
            updateTractorBeam(timestamp);
            return;
            */
        default:
            System.out.println("Unexpected drive control state: " + mDriveControlState);
            break;
        }
      }
    }

    
    @Override
    public void onStop(double timestamp) {
      synchronized(SwerveDrive.this){
        stop();
        mCSVWriter.flush();
      }
    }
  };

/**
     * Drives the robot straight forwards until it is at an optimal shooting distance. Then sends the robot into the
     * AIM_TO_GOAL state for one final alignment
     */
    private void updateDriveTowardsGoalApproach(double timestamp) {
      Optional<ShooterAimingParameters> aim = mRobotState.getAimingParameters();
      mIsApproaching = true;
      if (aim.isPresent()) {
          final double distance = aim.get().getRange();
          double error = 0.0;
          if (distance < Constants.kShooterOptimalRangeFloor) {
              error = distance - Constants.kShooterOptimalRangeFloor;
          } else if (distance > Constants.kShooterOptimalRangeCeiling) {
              error = distance - Constants.kShooterOptimalRangeCeiling;
          }
          final double kGoalPosTolerance = 1.0; // inches
          if (Util.epsilonEquals(error, 0.0, kGoalPosTolerance)) {
              // We are on target. Switch back to auto-aim.
              mDriveControlState = DriveControlState.AIM_TO_GOAL;
              RobotState.getInstance().resetVision();
              mIsApproaching = false;
              updatePositionSetpoint(getLeftDistanceInches(), getRightDistanceInches());
              return;
          }
          updatePositionSetpoint(getLeftDistanceInches() + error, getRightDistanceInches() + error);
      } else {
          updatePositionSetpoint(getLeftDistanceInches(), getRightDistanceInches());
      }
  }

    /**
     * Essentially does the same thing as updateTurnToHeading but sends the robot into the DRIVE_TOWARDS_GOAL_APPROACH
     * state if it detects we are not at an optimal shooting range
     */
    private void updateDriveTowardsGoalCoarseAlign(double timestamp) {
      updateGoalHeading(timestamp);
      updateTurnToHeading(timestamp);
      mIsApproaching = true;
      if (mIsOnTarget) {
          // Done coarse alignment.

          Optional<ShooterAimingParameters> aim = mRobotState.getAimingParameters();
          if (aim.isPresent()) {
              final double distance = aim.get().getRange();

              if (distance < Constants.kShooterOptimalRangeCeiling &&
                      distance > Constants.kShooterOptimalRangeFloor) {
                  // Don't drive, just shoot.
                  mDriveControlState = DriveControlState.AIM_TO_GOAL;
                  mIsApproaching = false;
                  mIsOnTarget = false;
                  updatePositionSetpoint(getLeftDistanceInches(), getRightDistanceInches());
                  return;
              }
          }

          mDriveControlState = DriveControlState.DRIVE_TOWARDS_GOAL_APPROACH;
          mIsOnTarget = false;
      }
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

  private static double rotationsToInches(double rotations) {
    return rotations * (Constants.kDriveWheelDiameterInches * Math.PI);

}

private static double rpmToInchesPerSecond(double rpm) {
    return rotationsToInches(rpm) / 60;
}

  /**
   * 
   * @deprecated This was used for the old tank drive. We need to make new functions for the new swerve drive
   */
  @Deprecated
  public double getLeftVelocityInchesPerSec() {
    //TODO: AAAAAAAAAAAA THIS IS WRONG
    return rpmToInchesPerSecond((wheels[0].getDriveEncoderVelocity()+wheels[2].getDriveEncoderVelocity())/2);
    //return 1; //rpmToInchesPerSecond((mLeftMaster.getSelectedSensorVelocity(Constants.kPidIdx)*(600.0/4096.0)));
  }

  /**
   * 
   * @deprecated This was used for the old tank drive. We need to make new functions for the new swerve drive
   */
  @Deprecated
  public double getRightVelocityInchesPerSec() {
    //TODO: AAAAAAAAAA THIS IS ALSO WRONG
    return rpmToInchesPerSecond((wheels[1].getDriveEncoderVelocity()+wheels[3].getDriveEncoderVelocity())/2);
    //return 1; //rpmToInchesPerSecond((mRightMaster.getSelectedSensorVelocity(Constants.kPidIdx)*(600.0/4096.0)));
  }

/**
   * 
   * @deprecated This was used for the old tank drive. We need to make new functions for the new swerve drive
   */
  @Deprecated
  public double getLeftDistanceInches() {
    //TODO: BBBBBBBBBBBB THIS IS BAD
    return rpmToInchesPerSecond((wheels[0].getDriveEncoderPos()+wheels[2].getDriveEncoderPos())/2);
    //return 1; //rotationsToInches(mLeftMaster.getSelectedSensorPosition(Constants.kPidIdx)/4096.0);
}

/**
   * 
   * @deprecated This was used for the old tank drive. We need to make new functions for the new swerve drive
   */
  @Deprecated
public double getRightDistanceInches() {
  // TODO: BBBBBBBBBB THIS IS VERY BAD
  return rpmToInchesPerSecond((wheels[1].getDriveEncoderPos()+wheels[3].getDriveEncoderPos())/2);
    //return 1; //rotationsToInches(mRightMaster.getSelectedSensorPosition(Constants.kPidIdx)/4096.0);
}

  /**
     * Turn the robot to a target heading.
     * 
     * Is called periodically when the robot is auto-aiming towards the boiler.
     */
    private void updateTurnToHeading(double timestamp) {
      final Rotation2d field_to_robot = mRobotState.getLatestFieldToVehicle().getValue().getRotation();

      // Figure out the rotation necessary to turn to face the goal.
      final Rotation2d robot_to_target = field_to_robot.inverse().rotateBy(mTargetHeading);

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
  }

/**
     * Check if the drive talons are configured for position control
     */
    protected static boolean usesTalonPositionControl(DriveControlState state) {
      if (state == DriveControlState.AIM_TO_GOAL ||
              state == DriveControlState.TURN_TO_HEADING ||
              state == DriveControlState.DRIVE_TOWARDS_GOAL_COARSE_ALIGN ||
              state == DriveControlState.DRIVE_TOWARDS_GOAL_APPROACH ||
              state == DriveControlState.CLIMB_BACKUP) {
          return true;
      }
      return false;
  }

  /**
     * Adjust position setpoint (if already in position mode)
     * 
     * @param left_inches_per_sec
     * @param right_inches_per_sec
     */
    private synchronized void updatePositionSetpoint(double left_position_inches, double right_position_inches) {
      //TODO: CCCCCCCCCCCCCC BAD
      if (usesTalonPositionControl(mDriveControlState)) {
        //mLeftMaster.set(ControlMode.MotionMagic, inchesToRotations(left_position_inches)*4096);
        //mRightMaster.set(ControlMode.MotionMagic, inchesToRotations(right_position_inches)*4096);
      } else {
          System.out.println("Hit a bad position control state");
          //mLeftMaster.set(ControlMode.MotionMagic,0);
          //mRightMaster.set(ControlMode.MotionMagic,0);
      }
  }

  /**
     * Called periodically when the robot is in path following mode. Updates the path follower with the robots latest
     * pose, distance driven, and velocity, the updates the wheel velocity setpoints.
     */
    private void updatePathFollower(double timestamp) {
        RigidTransform2d robot_pose = mRobotState.getLatestFieldToVehicle().getValue();
        Twist2d command = mPathFollower.update(timestamp, robot_pose,
                RobotState.getInstance().getDistanceDriven(), RobotState.getInstance().getPredictedVelocity().dx);
        if (!mPathFollower.isFinished()) {

            SmartDashboard.putNumber("Auto Drive dx", command.dx);
            SmartDashboard.putNumber("Auto Drive dtheta", command.dtheta);
            
            //Convert polar to coordinates. Supposedly dtheta is an angle and dx is a magnitude.
            //Easily could be wrong...
            //Source: https://www.analyzemath.com/polarcoordinates/polar_rectangular.html
            double R = command.dx;
            double t = Math.toDegrees(command.dtheta);
            //x = R * cos(t)
            //y = R * sin(t)

            SmartDashboard.putNumber("R (dx)", R);
            SmartDashboard.putNumber("t (dtheta)", t);

            double autoforward = R * Math.cos(t);
            double autostrafe = R * Math.sin(t);

            SmartDashboard.putNumber("Auto Drive Forward", autoforward);
            SmartDashboard.putNumber("Auto Drive Strafe", autostrafe);

            drive(autoforward, autostrafe, 0);

            Kinematics.DriveVelocity setpoint = Kinematics.inverseKinematics(command);
            updateVelocitySetpoint(setpoint.left, setpoint.right);
        } else {
            updateVelocitySetpoint(0, 0);
        }
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
    public synchronized void setWantTurnToHeading(Rotation2d heading) {
      if (mDriveControlState != DriveControlState.TURN_TO_HEADING) {
          configureTalonsForPositionControl();
          mDriveControlState = DriveControlState.TURN_TO_HEADING;
          updatePositionSetpoint(getLeftDistanceInches(), getRightDistanceInches());
      }
   //   if (Math.abs(heading.inverse().rotateBy(mTargetHeading).getDegrees()) > 1E-3) {
          mTargetHeading = heading;
          mIsOnTarget = false;
   //   }
   //   setHighGear(false);
  }

  /**
     * Configures talons for position control
     */
    private void configureTalonsForPositionControl() {
      //TODO: EEEEEEEEEEEEEEEEE That's not right!
      if (!usesTalonPositionControl(mDriveControlState)) {
        /*
        mLeftMaster.set(ControlMode.MotionMagic, 0);
        mLeftMaster.configNominalOutputForward(Constants.kDriveLowGearNominalOutput,Constants.kTimeoutMs);
        mLeftMaster.configNominalOutputReverse(-Constants.kDriveLowGearNominalOutput, Constants.kTimeoutMs);
        mLeftMaster.selectProfileSlot(kLowGearPositionControlSlot, Constants.kPidIdx);
        mRightMaster.set(ControlMode.MotionMagic, 0);
        mRightMaster.configNominalOutputForward(Constants.kDriveLowGearNominalOutput,Constants.kTimeoutMs);
        mRightMaster.configNominalOutputReverse(-Constants.kDriveLowGearNominalOutput, Constants.kTimeoutMs);
        mRightMaster.selectProfileSlot(kLowGearPositionControlSlot, Constants.kPidIdx);
          setBrakeMode(true);
          */
      }
  }

    /**
     * Start up velocity mode. This sets the drive train in high gear as well.
     * 
     * @param left_inches_per_sec
     * @param right_inches_per_sec
     */
    public synchronized void setVelocitySetpoint(double left_inches_per_sec, double right_inches_per_sec) {
      configureTalonsForSpeedControl();
      mDriveControlState = DriveControlState.VELOCITY_SETPOINT;
      updateVelocitySetpoint(left_inches_per_sec, right_inches_per_sec);
  }

  private void configureTalonsForSpeedControl() {
    if (!usesTalonVelocityControl(mDriveControlState)) {
      /*
      mLeftMaster.set(ControlMode.Velocity, 2);
      mLeftMaster.configNominalOutputForward(Constants.kDriveHighGearNominalOutput,Constants.kTimeoutMs);
      mLeftMaster.configNominalOutputReverse(-Constants.kDriveHighGearNominalOutput, Constants.kTimeoutMs);
      mLeftMaster.selectProfileSlot(kHighGearVelocityControlSlot, Constants.kPidIdx);

        mRightMaster.set(ControlMode.Velocity, 2);
      mRightMaster.configNominalOutputForward(Constants.kDriveHighGearNominalOutput,Constants.kTimeoutMs);
      mRightMaster.configNominalOutputReverse(-Constants.kDriveHighGearNominalOutput, Constants.kTimeoutMs);
      mRightMaster.selectProfileSlot(kHighGearVelocityControlSlot, Constants.kPidIdx);
        setBrakeMode(true);
        */
    }
}

  /**
     * Adjust Velocity setpoint (if already in velocity mode)
     * 
     * @param left_inches_per_sec
     * @param right_inches_per_sec
     */
    private synchronized void updateVelocitySetpoint(double left_inches_per_sec, double right_inches_per_sec) {
      if (usesTalonVelocityControl(mDriveControlState)) {
          final double max_desired = Math.max(Math.abs(left_inches_per_sec), Math.abs(right_inches_per_sec));
          final double scale = max_desired > Constants.kDriveHighGearMaxSetpoint
                  ? Constants.kDriveHighGearMaxSetpoint / max_desired : 1.0;

              //TODO: Drive math
              /*
                So this code was written for tank drive. Turning is defined by decreasing a side. Of course, we don't
                want to do this, turning will be defined by an azimuth parameter. We want to just translate along the path instead
                To do this, we must split the two side direction variables into a forward and strafe direction...

                First thing's first, we need to follow the path. We can fix up azimuth direction later.

                The robot faces the positive X direction. Positive Y is to the left of the robot. So therefore

                Forward is X
                Strafe is Y

                This is especially true when we are field oriented. No matter which direction the robot is facing, forward
                is always X and strafe is always Y.

                Perhaps a ratio? Say we need to turn left...

                left = 0.5
                right = 1
                
                left/right = 0.5/1 = 0.5

                right/left = 1/0.5 = 2

                or say we turn right... should just be the obverse

                left = 1
                right = 0.5

                left/right = 1/0.5 = 2

                right/left = 0.5/1 = 0.5

                Yeah... so I'm going to run with left/right. 

                This means that a number larger than 1 is turning right, while a number smaller than 1 but larger than 0 is turning left.

                So... max_desired * left/right would be the strafe?
              */


            //mLeftMaster.set(ControlMode.Velocity, inchesPerSecondToUnitsPer100ms(left_inches_per_sec * scale));
            //mRightMaster.set(ControlMode.Velocity, inchesPerSecondToUnitsPer100ms(right_inches_per_sec * scale));
      } else {
          System.out.println("Hit a bad velocity control state");
          //mLeftMaster.set(ControlMode.Velocity,0);
          //mRightMaster.set(ControlMode.Velocity,0);
      }

  }

  private static double inchesPerSecondToUnitsPer100ms(double inches_per_second) {
    return ((inchesPerSecondToRpm(inches_per_second))*(4096.0)/600.0);
  }

  private static double inchesPerSecondToRpm(double inches_per_second) {
    return inchesToRotations(inches_per_second) * 60;
}

private static double inchesToRotations(double inches) {
  return inches / (Constants.kDriveWheelDiameterInches * Math.PI);
}

  /**
     * Check if the drive talons are configured for velocity control
     */
    protected static boolean usesTalonVelocityControl(DriveControlState state) {
      if (state == DriveControlState.VELOCITY_SETPOINT || state == DriveControlState.PATH_FOLLOWING || state == DriveControlState.TRACTOR_BEAM) {
          return true;
      }
      return false;
  }

  public SwerveDrive(SwerveDriveConfig config) {
    // logger.info("<b>SwerveDrive</b>: SwerveDrive starting");
    m_motor = new CANSparkMax(0, MotorType.kBrushless);
    m_motor.disable();

    gyro = config.gyro;
    wheels = config.wheels;

    final boolean summarizeErrors = config.summarizeTalonErrors;
    Errors.setSummarized(summarizeErrors);
    Errors.setCount(0);
    // logger.debug("TalonSRX configuration errors summarized = {}",
    // summarizeErrors);

    double length = config.length;
    double width = config.width;
    double radius = Math.hypot(length, width);
    kLengthComponent = length / radius;
    kWidthComponent = width / radius;

    logger.info("gyro is configured: {}", gyro != null);
    logger.info("gyro is connected: {}", gyro != null && gyro.isConnected());
    setFieldOriented(gyro != null && gyro.isConnected());

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

    mNavXBoard = new NavX(Port.kMXP);

    // logger.info("<b>SwerveDrive</b>: SwerveDrive constructed");
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
    // logger.info("<b>SwerveDrive</b>: setDriveMode starting");
    for (Wheel wheel : wheels) {
      wheel.setDriveMode(driveMode);
    }
    logger.info("drive mode = {}", driveMode);
    logger.info("gyro is configured: {}", gyro != null);
    logger.info("gyro is connected: {}", gyro != null && gyro.isConnected());
    if (isFieldOriented) {
      logger.info("ROBOT IS FIELD ORIENTED IN setDriveMode()");
    }
    // logger.info("<b>SwerveDrive</b>: setDriveMode finished");
  }

  /**
   * Set all four wheels to specified values.
   *
   * @param azimuth -0.5 to 0.5 rotations, measured clockwise with zero being the
   *                robot straight-ahead position
   * @param drive   0 to 1 in the direction of the wheel azimuth
   */
  public void set(double azimuth, double drive) {
    // logger.info("<b>SwerveDrive</b>: set starting");
    for (Wheel wheel : wheels) {
      wheel.set(azimuth, drive);
    }
    // logger.info("<b>SwerveDrive</b>: set finished");
  }

  public synchronized Rotation2d getGyroAngle() {
    return mNavXBoard.getYaw();
}

public synchronized NavX getNavXBoard() {
  return mNavXBoard;
}

public synchronized void setGyroAngle(Rotation2d angle) {
  mNavXBoard.reset();
  mNavXBoard.setAngleAdjustment(angle);
}

public synchronized double getGyroVelocityDegreesPerSec() {
  return mNavXBoard.getYawRateDegreesPerSec();
}

  /**
   * Drive the robot in given field-relative direction and with given rotation.
   *
   * @param forward Y-axis movement, from -1.0 (reverse) to 1.0 (forward)
   * @param strafe  X-axis movement, from -1.0 (left) to 1.0 (right)
   * @param azimuth robot rotation, from -1.0 (CCW) to 1.0 (CW)
   */
  public void drive(double forward, double strafe, double azimuth) {
    // logger.info("<b>SwerveDrive</b>: drive starting");

    // Use gyro for field-oriented drive. We use getAngle instead of getYaw to
    // enable arbitrary
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
    // logger.info("<b>SwerveDrive</b>: drive finished");
  }

  /**
   * Stops all wheels' azimuth and drive movement. Calling this in the robots
   * {@code teleopInit} and {@code autonomousInit} will reset wheel azimuth
   * relative encoders to the current position and thereby prevent wheel rotation
   * if the wheels were moved manually while the robot was disabled.
   */
  public void stopWheels() {
    // logger.info("<b>SwerveDrive</b>: stop starting");
    for (Wheel wheel : wheels) {
      wheel.stop();
    }
    logger.info("stopped all wheels");

    // logger.info("<b>SwerveDrive</b>: stop finished");
  }

/**
     * Configure talons for open loop control
     */
    public synchronized void setOpenLoop(DriveSignal signal) {
      //TODO: DDDDDDDDD VHERE DID VWE GOE SOOOOO VWRONGGG
      if (mDriveControlState != DriveControlState.OPEN_LOOP) {
        /*
          mLeftMaster.set(ControlMode.PercentOutput, signal.getLeft());
          mRightMaster.set(ControlMode.PercentOutput, signal.getRight());
          mLeftMaster.configNominalOutputForward(0, 0);
          mLeftMaster.configNominalOutputReverse(0, 0);
          mRightMaster.configNominalOutputForward(0, 0);
          mRightMaster.configNominalOutputReverse(0, 0);
          */
          mDriveControlState = DriveControlState.OPEN_LOOP;
          //setBrakeMode(false);
      }
      // Right side is reversed, but reverseOutput doesn't invert PercentVBus.
      // So set negative on the right master.

   // DriverStation.reportError("setOpenLoop" + signal, false);
      //mRightMaster.set(ControlMode.PercentOutput, signal.getRight());
      //mLeftMaster.set(ControlMode.PercentOutput, signal.getLeft());
  }

  @Override
  public synchronized void stop(){
    stopWheels();

    setOpenLoop(DriveSignal.NEUTRAL);
  }

  /**
   * Save the wheels' azimuth current position as read by absolute encoder. These
   * values are saved persistently on the roboRIO and are normally used to
   * calculate the relative encoder offset during wheel initialization.
   *
   * <p>
   * The wheel alignment data is saved in the WPI preferences data store and may
   * be viewed using a network tables viewer.
   *
   * @see #zeroAzimuthEncoders()
   */
  public void saveAzimuthPositions() {
    saveAzimuthPositions(Preferences.getInstance());
  }

  void saveAzimuthPositions(Preferences prefs) {
    // logger.info("<b>SwerveDrive</b>: saveAzimuthPositions starting");
    for (int i = 0; i < WHEEL_COUNT; i++) {
      int position = wheels[i].getAzimuthAbsolutePosition();
      prefs.putInt(getPreferenceKeyForWheel(i), position);
      logger.info("azimuth {}: saved zero = {}", i, position);
    }
    // logger.info("<b>SwerveDrive</b>: saveAzimuthPositions finished");
  }

  /**
   * Set wheels' azimuth relative offset from zero based on the current absolute
   * position. This uses the physical zero position as read by the absolute
   * encoder and saved during the wheel alignment process.
   *
   * @see #saveAzimuthPositions()
   */
  public void zeroAzimuthEncoders() {
    zeroAzimuthEncoders(Preferences.getInstance());
  }

  void zeroAzimuthEncoders(Preferences prefs) {
    // logger.info("<b>SwerveDrive</b>: zeroAzimuthEncoders starting");
    Errors.setCount(0);
    for (int i = 0; i < WHEEL_COUNT; i++) {
      int position = prefs.getInt(getPreferenceKeyForWheel(i), DEFAULT_ABSOLUTE_AZIMUTH_OFFSET);
      wheels[i].setAzimuthZero(position);
      logger.info("azimuth {}: loaded zero = {}", i, position);
    }
    int errorCount = Errors.getCount();
    if (errorCount > 0)
      logger.error("TalonSRX set azimuth zero error count = {}", errorCount);

    // logger.info("<b>SwerveDrive</b>: zeroAzimuthEncoders finished");
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
   * Enable or disable field-oriented driving. Enabled by default if connected
   * gyro is passed in via {@code SwerveDriveConfig} during construction.
   *
   * @param enabled true to enable field-oriented driving.
   */
  public void setFieldOriented(boolean enabled) {
    isFieldOriented = enabled;
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
    OPEN_LOOP, CLOSED_LOOP, TELEOP, TRAJECTORY, AZIMUTH
  }

/**
     * Configures the drivebase to drive a path. Used for autonomous driving
     * 
     * @see Path
     */
    public synchronized void setWantDrivePath(Path path, boolean reversed) {
      if (mCurrentPath != path || mDriveControlState != DriveControlState.PATH_FOLLOWING) {
          configureTalonsForSpeedControl();
          RobotState.getInstance().resetDistanceDriven();
          mPathFollower = new PathFollower(path, reversed,
                  new PathFollower.Parameters(
                          new Lookahead(Constants.kMinLookAhead, Constants.kMaxLookAhead,
                                  Constants.kMinLookAheadSpeed, Constants.kMaxLookAheadSpeed),
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

  @Override
  public void outputToSmartDashboard() {
    // TODO Auto-generated method stub
    SmartDashboard.putString("SwerveStatus", "AASSHZHIAAA");
  }

  @Override
  public void zeroSensors() {
    // TODO Auto-generated method stub
    zeroAzimuthEncoders();
    mNavXBoard.reset();
    mNavXBoard.setAngleAdjustment( Rotation2d.fromDegrees(0.0));
  }

  @Override
  public void registerEnabledLoops(Looper enabledLooper) {
    // TODO Auto-generated method stub
    enabledLooper.register(mLoop);
  }
}
