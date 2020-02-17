/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DoubleSolenoid;

import edu.wpi.first.wpilibj.PWMTalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;

import frc.robot.Constants;
import frc.robot.Constants.OpConstants;

public class ShootClimbSubsystem extends SubsystemBase {

  private DoubleSolenoid mShootClimbSolenoid;
  private DoubleSolenoid mClimberSolenoid;
  private DoubleSolenoid mShootHoodSolenoid;
  //private final PWMTalonFX mTalonShoot;
  private final TalonFX mTalonShoot1;
  private final TalonFX mTalonShoot2;
  private DigitalOutput mColor1;
  private DigitalOutput zMode;

  private boolean modeClimbing;
  private double extendRetract;
  
  /**
   * Creates a new ExampleSubsystem.
   */
  public ShootClimbSubsystem() {
    mShootClimbSolenoid = Constants.makeDoubleSolenoidForIds(0, OpConstants.k0Shooting, OpConstants.k0Climbing);
    mClimberSolenoid = Constants.makeDoubleSolenoidForIds(1, OpConstants.k1ClimbRetract, OpConstants.k1ClimbExtend);
    mShootHoodSolenoid = Constants.makeDoubleSolenoidForIds(1, OpConstants.k1HoodRetract, OpConstants.k1HoodExtend);
    //mTalonShoot = new PWMTalonFX(OpConstants.kMotorPWMShoot1);
    mTalonShoot1 = new TalonFX(OpConstants.kMotorCANShoot1);
    mTalonShoot2 = new TalonFX(OpConstants.kMotorCANShoot2);

    mTalonShoot1.configFactoryDefault();
    mTalonShoot2.configFactoryDefault();

    //make both shooter motors run
    mTalonShoot2.follow(mTalonShoot1);
    mTalonShoot2.setInverted(TalonFXInvertType.OpposeMaster);

    /* Config sensor used for Primary PID [Velocity] */
    mTalonShoot1.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, OpConstants.kPIDLoopIdx, OpConstants.kTimeoutMs);
    mTalonShoot2.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, OpConstants.kPIDLoopIdx, OpConstants.kTimeoutMs);
    /** Phase sensor accordingly. 
      * Positive Sensor Reading should match Green (blinking) Leds on Talon
    */
		mTalonShoot1.setSensorPhase(true);
		mTalonShoot2.setSensorPhase(true);

		/* Config the peak and nominal outputs */
		mTalonShoot1.configNominalOutputForward(0, OpConstants.kTimeoutMs);
		mTalonShoot1.configNominalOutputReverse(0, OpConstants.kTimeoutMs);
		mTalonShoot1.configPeakOutputForward(1, OpConstants.kTimeoutMs);
		mTalonShoot1.configPeakOutputReverse(-1, OpConstants.kTimeoutMs);
		mTalonShoot2.configNominalOutputForward(0, OpConstants.kTimeoutMs);
		mTalonShoot2.configNominalOutputReverse(0, OpConstants.kTimeoutMs);
		mTalonShoot2.configPeakOutputForward(1, OpConstants.kTimeoutMs);
		mTalonShoot2.configPeakOutputReverse(-1, OpConstants.kTimeoutMs);

		/* Config the Velocity closed loop gains in slot0 */
		mTalonShoot1.config_kF(OpConstants.kPIDLoopIdx, OpConstants.kGains_Velocity.kF, OpConstants.kTimeoutMs);
		mTalonShoot1.config_kP(OpConstants.kPIDLoopIdx, OpConstants.kGains_Velocity.kP, OpConstants.kTimeoutMs);
		mTalonShoot1.config_kI(OpConstants.kPIDLoopIdx, OpConstants.kGains_Velocity.kI, OpConstants.kTimeoutMs);
		mTalonShoot1.config_kD(OpConstants.kPIDLoopIdx, OpConstants.kGains_Velocity.kD, OpConstants.kTimeoutMs);
		mTalonShoot2.config_kF(OpConstants.kPIDLoopIdx, OpConstants.kGains_Velocity.kF, OpConstants.kTimeoutMs);
		mTalonShoot2.config_kP(OpConstants.kPIDLoopIdx, OpConstants.kGains_Velocity.kP, OpConstants.kTimeoutMs);
		mTalonShoot2.config_kI(OpConstants.kPIDLoopIdx, OpConstants.kGains_Velocity.kI, OpConstants.kTimeoutMs);
		mTalonShoot2.config_kD(OpConstants.kPIDLoopIdx, OpConstants.kGains_Velocity.kD, OpConstants.kTimeoutMs);

    modeClimbing = false;
    extendRetract = 0;

    mColor1 = new DigitalOutput(7);
    zMode = new DigitalOutput(5);
  }

  public void testSpeed(){
    System.out.println("testSpeed");
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (modeClimbing) {
      // Climb mode
    } // else Climbing mode
  }

  public void enableShooting() {
    mColor1.set(true);
    //mShootClimbSolenoid.set(DoubleSolenoid.Value.kReverse);
    //mTalonShoot.setSpeed(0.7);
    /**
			 * Convert 500 RPM to units / 100ms.
			 * 2048(FX) 4096(SRX) Units/Rev * 500 RPM / 600 100ms/min in either direction:
			 * velocity setpoint is in units/100ms
    */
    double velocity = 0.5; // guessing between -1.0 to 1.0
		double targetVelocity_UnitsPer100ms = velocity * 500.0 * 2048 / 600;
		/* 500 RPM in either direction */
		//mTalonShoot1.set(ControlMode.Velocity, targetVelocity_UnitsPer100ms);
    mTalonShoot1.set(ControlMode.PercentOutput,OpConstants.kMotorShootPercent);
    //mTalonShoot2.set(ControlMode.PercentOutput,OpConstants.kMotorShootPercent);
  }
  public void enableClimbing() {
    mColor1.set(false);
    //mShootClimbSolenoid.set(DoubleSolenoid.Value.kForward);
    //mTalonShoot.setSpeed(0);
    mTalonShoot1.set(ControlMode.PercentOutput,OpConstants.kMotorShootPercent);
    //mTalonShoot2.set(ControlMode.PercentOutput,OpConstants.kMotorShootPercent);
  }

  public void climbExtend() {
    if (modeClimbing) {
      //mClimberSolenoid.set(DoubleSolenoid.Value.kForward);
    }
  }

  public void climbRetract() {
    if (modeClimbing) {
     // mClimberSolenoid.set(DoubleSolenoid.Value.kReverse);
    }
  }

  public void disable() {
    //mShootClimbSolenoid.set(DoubleSolenoid.Value.kReverse);
    //mClimberSolenoid.set(DoubleSolenoid.Value.kReverse);
    //mShootHoodSolenoid.set(DoubleSolenoid.Value.kReverse);
    //mTalonShoot.setSpeed(0);
    mTalonShoot1.set(ControlMode.PercentOutput, 0);
    mTalonShoot2.set(ControlMode.PercentOutput, 0);
  }

  public void modeShoot() {
    modeClimbing = false;
    zMode.set(false);
    //mTalonShoot.setSpeed(0.5);
    mTalonShoot1.set(ControlMode.PercentOutput,OpConstants.kMotorShootPercent);
    //mTalonShoot2.set(ControlMode.PercentOutput,OpConstants.kMotorShootPercent);
  }

  public void modeClimb() {
    modeClimbing = true;
    zMode.set(true);
    //mTalonShoot.setSpeed(-0.0); // for testing only
    mTalonShoot1.set(ControlMode.PercentOutput,OpConstants.kMotorShootPercent);
    //mTalonShoot2.set(ControlMode.PercentOutput, 0);
  }

  public void hoodRetract() {
    //mShootHoodSolenoid.set(DoubleSolenoid.Value.kReverse);
  }

  public void hoodExtend() {
    //mShootHoodSolenoid.set(DoubleSolenoid.Value.kForward);
  }

  public void setClimber(double zVal) {
    if (modeClimbing) {
      extendRetract = zVal;
    }
  }
}
