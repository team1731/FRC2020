/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;
import edu.wpi.first.wpilibj.DoubleSolenoid;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

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

    //
    // PCM 0 SOLENOIDS
    //
    public static final int kIntakeExtend = 0; 
    public static final int kIntakeRetract = 1; 
    public static final int kShooting = 2; 
    public static final int kClimbing = 3; 
    public static final int kClimbExtend = 4; 
    public static final int kClimbRetract = 5; 
    public static final int kHoodExtend = 6; 
    public static final int kHoodRetract = 7; 

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

        //
    // PCM 0 SOLENOIDS
    //
    public static final int kBeakSwinger1 = 4; 
    public static final int kBeakSwinger2 = 5; 
    public static final int kMustache1 = 7; 
    public static final int kMustache2 = 6; 
    public static final int kDSolenoidExtra1 = 0;
    public static final int kDSolenoidExtra2 = 1;

    //
    // PCM 1 SOLENOIDS
    //
    public static final int kBeakOpener1 = 0;
    public static final int kBeakOpener2 = 5;
    
    public static final int kRotateWristShort1 = 2;
    public static final int kRotateWristShort2 = 4;
    
    public static final int kRotateWristLong1 = 3;
    public static final int kRotateWristLong2 = 1;
    
    public static final int kTopRoller1 = 7;
    public static final int kTopRoller2 = 6;

    public static DoubleSolenoid makeDoubleSolenoidForIds(int pcmChannel, int forward_solenoidId, int reverse_solenoidId) {
    	System.out.println("creating solenoid ids " + forward_solenoidId + "-" + reverse_solenoidId + " PCM " + pcmChannel + " CHAN ");
        return new DoubleSolenoid(pcmChannel, forward_solenoidId, reverse_solenoidId);
    }

}
