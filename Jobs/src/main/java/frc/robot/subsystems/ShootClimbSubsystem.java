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
import frc.robot.Constants;

public class ShootClimbSubsystem extends SubsystemBase {

  private final DoubleSolenoid mShootClimbSolenoid;
  private final DoubleSolenoid mClimberSolenoid;
  private final DoubleSolenoid mShootHoodSolenoid;
  private final PWMTalonFX mTalonShoot;
  //private final TalonFX mTalonShoot1;
  //private final TalonFX mTalonShoot2;
  private DigitalOutput mColor1;
  private boolean modeClimbing;
  
  /**
   * Creates a new ExampleSubsystem.
   */
  public ShootClimbSubsystem() {
    mShootClimbSolenoid = Constants.makeDoubleSolenoidForIds(0, Constants.kShooting, Constants.kClimbing);
    mClimberSolenoid = Constants.makeDoubleSolenoidForIds(0, Constants.kClimbRetract, Constants.kClimbExtend);
    mShootHoodSolenoid = Constants.makeDoubleSolenoidForIds(0, Constants.kHoodRetract, Constants.kHoodExtend);
    mTalonShoot = new PWMTalonFX(Constants.kMotorPWMShoot1);
    //mTalonShoot1 = new TalonFX(Constants.kMotorCANShoot1);
    //mTalonShoot2 = new TalonFX(Constants.kMotorCANShoot2);

    modeClimbing = false;

    mColor1 = new DigitalOutput(7);
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
    mShootClimbSolenoid.set(DoubleSolenoid.Value.kReverse);
    mTalonShoot.setSpeed(0);
    //mTalonShoot1.set(ControlMode.PercentOutput,Constants.kMotorShootPercent);
    //mTalonShoot2.set(ControlMode.PercentOutput,Constants.kMotorShootPercent);
  }
  public void enableClimbing() {
    mColor1.set(false);
    mShootClimbSolenoid.set(DoubleSolenoid.Value.kForward);
    mTalonShoot.setSpeed(0);
    //mTalonShoot1.set(ControlMode.PercentOutput,Constants.kMotorShootPercent);
    //mTalonShoot2.set(ControlMode.PercentOutput,Constants.kMotorShootPercent);
  }

  public void climbExtend() {
    if (modeClimbing) {
      mClimberSolenoid.set(DoubleSolenoid.Value.kForward);
    }
  }

  public void climbRetract() {
    if (modeClimbing) {
      mClimberSolenoid.set(DoubleSolenoid.Value.kReverse);
    }
  }

  public void disable() {
    mShootClimbSolenoid.set(DoubleSolenoid.Value.kReverse);
    mClimberSolenoid.set(DoubleSolenoid.Value.kReverse);
    mShootHoodSolenoid.set(DoubleSolenoid.Value.kReverse);
    mTalonShoot.setSpeed(0);
    //mTalonShoot1.set(ControlMode.PercentOutput, 0);
    //mTalonShoot2.set(ControlMode.PercentOutput, 0);
  }

  public void modeShoot() {
    modeClimbing = false;
    mTalonShoot.setSpeed(0.5);
    //mTalonShoot1.set(ControlMode.PercentOutput,Constants.kMotorShootPercent);
    //mTalonShoot2.set(ControlMode.PercentOutput,Constants.kMotorShootPercent);
  }

  public void modeClimb() {
    modeClimbing = true;
    mTalonShoot.setSpeed(-0.0); // for testing only
    //mTalonShoot1.set(ControlMode.PercentOutput,Constants.kMotorShootPercent);
    //mTalonShoot2.set(ControlMode.PercentOutput, 0);
  }

  public void hoodRetract() {
    mShootHoodSolenoid.set(DoubleSolenoid.Value.kReverse);
  }

  public void hoodExtend() {
    mShootHoodSolenoid.set(DoubleSolenoid.Value.kForward);
  }

}
