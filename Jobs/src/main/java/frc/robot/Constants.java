/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.util.Color;
import com.revrobotics.ColorMatch;
import frc.robot.Gains;

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

    public static final class OpConstants {
        // PWM
        public static final int kPWM_LedSting = 9;         // Addressable Led String

        // Intake
        public static final int kMotorPWMIntake = 0;      // Intake
        public static final int kMotorPWMSeq = 1;         // Sequencer
        public static final int kMotorPWMShoot1 = 2;       // Shooter Motor One
        public static final int kMotorPWMShoot2 = 3;       // Shooter Motor Two
        public static final int kMotorCANShoot1 = 5;
        public static final int kMotorCANShoot2 = 6;
        public static final double kMotorSeqFwdSpeed = -0.4;   // forward or backward
        public static final double kMotorSeqRevSpeed = 0.4;   // forward or backward
        public static final double kMotorIntakeFwdSpeed = 0.2;   // forward or backward
        public static final double kMotorIntakeRevSpeed = -0.2;   // forward or backward
        public static final double kMotorShootSpeed1 = -0.3;   // forward or backward
        public static final double kMotorShootSpeed2 = 0.3;
        public static final double kMotorShootPercent = 0.80;   // check shooting motor percent
        public static final double kMotorClimbPercent = 0.3;
        public static final int kMaxPowerCells = 3;

        // ColorWheel
        public static final int kColorWheelTalonFX = 8;

        // Shooter
        public static final int kShooterVictor = 3;

        // Digital Input/Outputs
        public static int kLowSequencer = 0;
        public static int kHighSequencer = 1;

        public static int kArduinoLed0 = 7;
        public static int kArduinoLed1 = 8;
        public static int kArduinoLed2 = 9;

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
            TEAM, RED, BLUE, GREEN, YELLOW, ORANGE, PURPLE, RAINBOW
        }

        // PCM 0 SOLENOIDS
        // pairs b4-t3, b5-t2, b6-t4, b7-t5, t0-t6, t1-t7
        public static final int kIntakeExtend = 0; 
        public static final int kIntakeRetract = 1; 
        public static final int kShooting = 2; 
        public static final int kClimbing = 3; 
        public static final int kClimbExtend = 4; 
        public static final int kClimbRetract = 5; 
        public static final int kHoodExtend = 6; 
        public static final int kHoodRetract = 7; 

        // PCM 1 SOLENOIDS
        public static final int kColorWheelExtend = 6; 
        public static final int kColorWheelRetract = 7; 
        // pairs b4-t3, b5-t2, b6-t4, b7-t5, t0-t6, t1-t7 

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
         * 	                                    			  kP   kI   kD   kF          Iz    PeakOut */
        public final static Gains kGains_Velocity = new Gains( 0.25, 0.001, 20, 1023.0/7200.0,  300,  1.00);
        ///// End TalonFX
    }
}
