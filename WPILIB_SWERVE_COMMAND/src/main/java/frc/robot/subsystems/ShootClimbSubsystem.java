/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;

// import edu.wpi.first.wpilibj.PWMTalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;

import frc.robot.Constants;
import frc.robot.Constants.OpConstants;
import frc.robot.Constants.OpConstants.LedOption;

public class ShootClimbSubsystem extends SubsystemBase {

  private final LedStringSubsystem m_ledstring;
  private DoubleSolenoid mShootClimbSolenoid;
  private DoubleSolenoid mClimberSolenoid;
  private DoubleSolenoid mShootHoodSolenoid;
  //private final PWMTalonFX mTalonShoot;
  private final TalonFX mTalonShoot1;
  private final TalonFX mTalonShoot2;

  //private boolean modeClimbing;
  
  /**
   * Creates a new ExampleSubsystem.
 * @param m_ledstring
   */
  public ShootClimbSubsystem(LedStringSubsystem m_ledstring) {
    this.m_ledstring = m_ledstring;
    mShootClimbSolenoid = Constants.makeDoubleSolenoidForIds(0, OpConstants.k0Shooting, OpConstants.k0Climbing);
    mClimberSolenoid = Constants.makeDoubleSolenoidForIds(1, OpConstants.k1ClimbRetract, OpConstants.k1ClimbExtend);
    mShootHoodSolenoid = Constants.makeDoubleSolenoidForIds(1, OpConstants.k1HoodRetract, OpConstants.k1HoodExtend);
    //mTalonShoot = new PWMTalonFX(OpConstants.kMotorPWMShoot1);
    mTalonShoot1 = new TalonFX(OpConstants.kMotorCANShoot1);
    mTalonShoot2 = new TalonFX(OpConstants.kMotorCANShoot2);

    mTalonShoot1.configFactoryDefault();
    mTalonShoot2.configFactoryDefault();

    //make both shooter motors run
    //mTalonShoot2.follow(mTalonShoot1);
    //mTalonShoot2.setInverted(TalonFXInvertType.OpposeMaster);
    mTalonShoot1.setInverted(TalonFXInvertType.CounterClockwise);
    mTalonShoot2.setInverted(TalonFXInvertType.Clockwise);

    /* Config sensor used for Primary PID [Velocity] */
    mTalonShoot1.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, OpConstants.kPIDLoopIdx, OpConstants.kTimeoutMs);
    mTalonShoot2.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, OpConstants.kPIDLoopIdx, OpConstants.kTimeoutMs);
    /** Phase sensor accordingly. 
      * Positive Sensor Reading should match Green (blinking) Leds on Talon
    */
		mTalonShoot1.setSensorPhase(false);
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

    shootMode();    

    SmartDashboard.putNumber("ShootingPercent", 0.5);
  }

  public void testSpeed(){
    System.out.println("testSpeed");
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //shootPercent = SmartDashboard.getNumber("ShootingPercent", 0.5);
    //enableShooting(shootPercent);
  }

  public double getShootMotor1Velocity() {
    return mTalonShoot1.getSelectedSensorVelocity();
  }
  
  public void enableShooting() {
    this.enableShooting(SmartDashboard.getNumber("ShootingPercent", 0.5));
  }
  
  public void enableShooting(double shootMotorPercent_0_to_1) {
    //mShootClimbSolenoid.set(DoubleSolenoid.Value.kReverse);
    //mTalonShoot.setSpeed(0.7);
    /**
			 * Convert 500 RPM to units / 100ms.
			 * 2048(FX) 4096(SRX) Units/Rev * 500 RPM / 600 100ms/min in either direction:
			 * velocity setpoint is in units/100ms
       * ==> 11425 is measured velocity at 80% / 0.8 = 9140/0.8
       * ==> 3347 is 11425 * 600 * 2048 == max speed in ticks per 100ms
       * ==> shootPercent is 0 to 1, so 100% == put in a value of 1.0
    */
    //double velocity = 0.1; // guessing between -1.0 to 1.0
		double targetVelocity_UnitsPer100ms = shootMotorPercent_0_to_1 * 3000.0 * 2048 / 600;
    /* 500 RPM in either direction */
		mTalonShoot1.set(ControlMode.Velocity, targetVelocity_UnitsPer100ms);
    mTalonShoot2.set(ControlMode.Velocity, targetVelocity_UnitsPer100ms);
    SmartDashboard.putNumber("bdltargetvelocity", targetVelocity_UnitsPer100ms);
    SmartDashboard.putNumber("talon1Velocity", mTalonShoot1.getSelectedSensorVelocity());
    SmartDashboard.putNumber("talon1Velocity2", mTalonShoot2.getSelectedSensorVelocity());

    //mTalonShoot1.set(ControlMode.PercentOutput, shootMotorPercent_0_to_1);
    //mTalonShoot2.set(ControlMode.PercentOutput, shootMotorPercent_0_to_1);
    hoodExtend();
    m_ledstring.option(LedOption.SHOOT);
  }

  public void stopShooting(){
    mTalonShoot1.set(ControlMode.PercentOutput, 0);
    mTalonShoot2.set(ControlMode.PercentOutput, 0);
    hoodRetract();
    shootMode();
  }

  public void enableClimbing() {
    //mShootClimbSolenoid.set(DoubleSolenoid.Value.kForward);
    //mTalonShoot.setSpeed(0);
    mTalonShoot1.set(ControlMode.PercentOutput,OpConstants.kMotorShootPercent);
    //mTalonShoot2.set(ControlMode.PercentOutput,OpConstants.kMotorShootPercent);
    m_ledstring.option(LedOption.CLIMB);
  }

  public void hoodRetract() {
    mShootHoodSolenoid.set(DoubleSolenoid.Value.kReverse);
  }

  public void hoodExtend() {
    mShootHoodSolenoid.set(DoubleSolenoid.Value.kForward);
  }

  public void setClimber(double percentOut) {
    double output = percentOut;
    // if within deadband then set output to Zero
    if (Math.abs(output) < OpConstants.kJoystickDeadband) {
      shootMode();
      output = 0;
    } else {
      climbMode();
    }
    //System.out.println("climb output = " + output);
    mTalonShoot1.set(ControlMode.PercentOutput, output*OpConstants.kClimbMaxPercent);
    mTalonShoot2.set(ControlMode.PercentOutput, output*OpConstants.kClimbMaxPercent);
  }


  public void shootMode() {
    mShootClimbSolenoid.set(DoubleSolenoid.Value.kForward);
  }

  public void climbMode() {
    mShootClimbSolenoid.set(DoubleSolenoid.Value.kReverse);
  }

  public void climbExtend() {
    // if (modeClimbing) {
    mClimberSolenoid.set(DoubleSolenoid.Value.kForward);
    // }
  }

  public void climbRetract() {
    // if (modeClimbing) {
    mClimberSolenoid.set(DoubleSolenoid.Value.kReverse);
    // }
  }

}

/*
  public void modeShoot() {
    modeClimbing = false;
    //mTalonShoot.setSpeed(0.5);
    mTalonShoot1.set(ControlMode.PercentOutput,OpConstants.kMotorShootPercent);
    //mTalonShoot2.set(ControlMode.PercentOutput,OpConstants.kMotorShootPercent);
  }

  public void modeClimb() {
    modeClimbing = true;
    //mTalonShoot.setSpeed(-0.0); // for testing only
    mTalonShoot1.set(ControlMode.PercentOutput,OpConstants.kMotorShootPercent);
    //mTalonShoot2.set(ControlMode.PercentOutput, 0);
  }

*/
