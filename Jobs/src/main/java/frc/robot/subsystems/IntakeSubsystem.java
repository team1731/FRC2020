/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.OpConstants;
import edu.wpi.first.wpilibj.PWMTalonFX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
//import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class IntakeSubsystem extends SubsystemBase {

  private final PWMTalonFX mTalonIntake;
  private final DoubleSolenoid mIntakeSolenoid;
  private String mTalonState;
  
  /**
   * Creates a new ExampleSubsystem.
   */
  public IntakeSubsystem() {
    mTalonIntake = new PWMTalonFX(OpConstants.kMotorPWMIntake);
    mIntakeSolenoid = Constants.makeDoubleSolenoidForIds(0, OpConstants.kIntakeRetract, OpConstants.kIntakeExtend);
    mTalonState = "Off";
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * Enables the intake by extending solenoid & turning on motor.
   */
  public void extend() {
    mIntakeSolenoid.set(DoubleSolenoid.Value.kForward);
    mTalonIntake.setSpeed(OpConstants.kMotorIntakeFwdSpeed);
    mTalonState = "Extending/Fwd";
  }

  /**
   * Enables the intake by extending solenoid & turning on motor.
   */
  public void eject() {
    //mIntakeSolenoid.set(true);
    mTalonIntake.setSpeed(OpConstants.kMotorIntakeRevSpeed);
    mTalonState = "Ejecting/Rev";
  }

  /**
   * Enables the intake by retracting solenoid & turning off motor.
   */
  public void retract() {
    mIntakeSolenoid.set(DoubleSolenoid.Value.kReverse);
    mTalonIntake.setSpeed(0);
    mTalonState = "Retracted/Off";
  }

  public String getIntakeState() {
    return(mTalonState);
  }
}
