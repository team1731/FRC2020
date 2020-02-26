/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.util.Color;
import com.revrobotics.ColorMatch;
//import frc.robot.Gains;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {


  public static DoubleSolenoid makeDoubleSolenoidForIds(int pcmChannel, int forward_solenoidId, int reverse_solenoidId) {
    System.out.println("creating solenoid ids " + forward_solenoidId + "-" + reverse_solenoidId + " PCM " + pcmChannel + " CHAN ");
    return new DoubleSolenoid(pcmChannel, forward_solenoidId, reverse_solenoidId);
  }
	
  public static final int kTICKS = 16;

  public static final class DriveConstants {

    //Drive motor CAN IDs
    public static final int kFrontLeftDriveMotorPort = 1;
    public static final int kFrontRightDriveMotorPort = 2;
    public static final int kRearLeftDriveMotorPort = 3;
    public static final int kRearRightDriveMotorPort = 4;

    //Turn motor CAN IDs
    public static final int kFrontLeftTurningMotorPort = 11;
    public static final int kFrontRightTurningMotorPort = 12;
    public static final int kRearLeftTurningMotorPort = 13;
    public static final int kRearRightTurningMotorPort = 14;

    public static final double kTrackWidth = 0.7112;
    //Distance between centers of right and left wheels on robot
    public static final double kWheelBase = 0.7;
    //Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics =
        new SwerveDriveKinematics(                                  // frontLeft, frontRight, rearLeft, rearRight
          new Translation2d(kWheelBase / 2, kTrackWidth / 2),
          new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
          new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
          new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    public static final boolean kGyroReversed = true; //09FEB false;

    public static final double kMaxTurnVelocity = 360;
    public static final double kMaxTurnAcceleration = 360;

    public static final double kTurnP = 0.05;
    public static final double kTurnI = 0;
    public static final double kTurnD = 0;

    public static final double kMinRightStickThreshold = 0.7;

    public static final double kTurnToleranceDeg = 5;
    public static final double kTurnRateToleranceDegPerS = 10; // degrees per second

    // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
    // These characterization values MUST be determined either experimentally or theoretically
    // for *your* robot's drive.
    // The RobotPy Characterization Toolsuite provides a convenient tool for obtaining these
    // values for your robot.
    public static final double ksVolts = 1;
    public static final double kvVoltSecondsPerMeter = 0.8;
    public static final double kaVoltSecondsSquaredPerMeter = 0.15;

    public static final double kMaxSpeedMetersPerSecond = 3;

  }


  public static final class ModuleConstants {
    public static final double kMaxModuleAngularSpeedRadiansPerSecond = 2 * Math.PI;
    public static final double kMaxModuleAngularAccelerationRadiansPerSecondSquared = 2 * Math.PI;

    public static final double kDriveEncoderCPR = 5.5;
    public static final double kTurningEncoderCPR = 16;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kDriveEncoderDistancePerPulse =
        // Assumes the encoders are directly mounted on the wheel shafts
        (kWheelDiameterMeters * Math.PI) / kDriveEncoderCPR;

        
    public static final double kTurningEncoderDistancePerPulse =
        // Assumes the encoders are on a 1:1 reduction with the module shaft.
        (2 * Math.PI) / kTurningEncoderCPR;

  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;

  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 1; //2
    public static final double kMaxAccelerationMetersPerSecondSquared = 1; //2
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    //Constraint for the motion profilied robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(kMaxAngularSpeedRadiansPerSecond,
          kMaxAngularSpeedRadiansPerSecondSquared);

  }
  
    public static final class OpConstants {
        // PWM
        public static final int kPWM_LedSting = 6;         // Addressable Led String

        // Intake
        public static final int kMotorPWMIntake = 0;      // Intake
        public static final int kMotorPWMSeq = 1;         // Sequencer
        public static final int kMotorPWMShoot1 = 2;       // Shooter Motor One
        public static final int kMotorPWMShoot2 = 3;       // Shooter Motor Two
        public static final int kMotorCANShoot1 = 7;
        public static final int kMotorCANShoot2 = 8;
        public static final double kMotorSeqFwdIntakeSpeed = -0.3;   // forward or backward
        public static final double kMotorSeqRevIntakeSpeed = 0.4;   // forward or backward
        public static final double kMotorSeqFwdShootSpeed = -0.4; //-0.5   // forward or backward
        public static final double kMotorSeqRevShootSpeed = 0.4;   // forward or backward
        public static final double kMotorIntakeFwdSpeed = 1.0;   // forward or backward
        public static final double kMotorIntakeRevSpeed = -1.0;   // forward or backward
        public static final double kMotorShootSpeed1 = -0.3;   // forward or backward
        public static final double kMotorShootSpeed2 = 0.3;
        public static final double kMotorShootPercent = 0.50;   // check shooting motor percent
        public static final double kMotorClimbPercent = 0.3;
        public static final int kMaxPowerCells = 5;
        public static final double kSeqResetDelay = 2.0;

        // ColorWheel
        public static final int kColorWheelTalonFX = 8;

        // Shooter
        public static final int kShooterVictor = 3;
        public static final int kShootMinVelocity = 500;

        public static final double kClimbMaxPercent = 1; // 0.3;
        public static final double kJoystickDeadband = 0.2;
        public static final double kClutchDeadband = 0.3;
        public static final int kClimbJoystickInvert = -1;

        public static final double kClimbExSafeEncValue = 10000;
        // Digital Input/Outputs
        public static int kLowSequencer = 0;
        public static int kMidSequencer = 1;
        public static int kHighSequencer = 2;

        public static int kHiCylinder = 3;
        public static int kLoCylinder = 4;
        public static int kClimbExtend = 5;
        public static int kClimbRetract = 6;

        //public static int kArduinoLed0 = 7;
        //public static int kArduinoLed1 = 8;
        //public static int kArduinoLed2 = 9;

        // Arduino Colors/Options
        public static int kArduino_TEAM  = 0;
        public static int kArduino_RED   = 1; // solid red
        public static int kArduino_GREEN = 2; // solid green
        public static int kArduino_BLUE  = 3; // solid blue
        public static int kArduino_REDW = 4; // red wipe
        public static int kArduino_GREENW = 5; // green wipe
        public static int kArduino_BLUEW = 6; // blue wipe
        public static int kArduino_YELLW = 7; // yellow wipe

        public enum LedOption {
          TEAM, RED, BLUE, GREEN, YELLOW, ORANGE, PURPLE, RAINBOW, FULL, CLIMB, SHOOT, INTAKE, INTAKEBALL, WHEEL, BALLONE, BALLTWO, BALLTHREE, BALLFOUR
        }
    
        // in order of pneumatic actuators (top to bottom)
        // PCM  terminals     function
        //  0      6-7    ==   spare
        //  0      4-5    ==   climb clutch
        //  1      0-1    ==   color wheel
        //  1      2-3    ==   climber arms
        //  1      4-5    ==   shooter hood
        //  1      6-7    ==   intake

        // PCM 0 SOLENOIDS
        public static final int k0SpareLeft = 6;
        public static final int k0SpareRight = 7;
        public static final int k0Shooting = 4; //ok
        public static final int k0Climbing = 5; //ok 

        // PCM 1 SOLENOIDS
        public static final int k1ColorWheelExtend = 0; 
        public static final int k1ColorWheelRetract = 1; 
        public static final int k1ClimbExtend = 2; 
        public static final int k1ClimbRetract = 3; 
        public static final int k1HoodExtend = 4; 
        public static final int k1HoodRetract = 5; 
        public static final int k1IntakeExtend = 6; //ok
        public static final int k1IntakeRetract = 7; //ok

        // ColorWheel
        // Note: Any example colors should be calibrated as the user needs, these are here as a basic example.
        public static Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
        public static Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
        public static Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
        public static Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);

        public static final int kWheelUnknown = 0;
        public static final int kWheelGreen = 1;
        public static final int kWheelBlue = 2;
        public static final int kWheelYellow= 3;
        public static final int kWheelRed = 4;

        public static final int kWheelCountRotate = 7;
        public static final int kWheelCountMatch = 1;

        public static double kWheelRotateSpeed = 0.5;
        public static double kWheelMatchFwdSpeed = 0.2;
        public static double kWheelMatchRevSpeed = -0.2;

        /////// TalonFX parameters
        public static final int kSlotIdx = 0;
        /**
         * Talon SRX/ Victor SPX will supported multiple (cascaded) PID loops. For
         * now we just want the primary one.
         */
        public static final int kPIDLoopIdx = 0;
        /**
         * Set to zero to skip waiting for confirmation, set to nonzero to wait and
         * report to DS if action fails.
         */
        public static final int kTimeoutMs = 30;
        /**
         * PID Gains may have to be adjusted based on the responsiveness of control loop.
         * kF: 1023 represents output value to Talon at 100%, 7200 represents Velocity units at 100% output
         * 
         * 	                                    		          	  kP   kI   kD   kF             Iz    PeakOut */
        public final static Gains kGains_Velocity = new Gains( 4.0, 0, 0, .07,  300,  1.00);
        ///// End TalonFX
    }

    public static final class VisionConstants {
        public static final double kBoilerTargetTopHeight = 27.5; //was 19.5
        public static final double kCameraZOffset = 4.0;  // they had 20.9
        public static final double kCameraDeadband = 0.0;
        public static final double kCameraPitchAngleDegrees = 18.135; // Measured on 3/30
        public static final double kCameraYawAngleDegrees = 0.0;
        public static final double kEpsilon = 1E-9;
        public static final double kMaxTrackerDistance = 18.0;
        public static final double kMaxGoalTrackAge = 1.0;  // cp had 1.0
        public static final double kCameraFrameRate = 30.0;
        public static final int kCameraBaudRate = 115200;
    }

    public static final class DriveConstantsOrig {
      public static final double kTurnP = 0.5;
      public static final double kTurnI = 0;
      public static final double kTurnD = 0;
      public static final double kTurnToleranceDeg = 5;
      public static final double kTurnRateToleranceDegPerS = 10; // degrees per second
      public static final double kMaxTurnRateDegPerS = 10;
      public static final double kMaxTurnAccelerationDegPerSSquared = 30;
    
    }
}
