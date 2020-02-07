/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.PWMTalonFX;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.DigitalOutput;

public class ShootClimbSubsystem extends SubsystemBase {

  private final PWMTalonFX mTalonShoot1;
  private DigitalOutput mColor1;
  private boolean mTest; // does robot want to index balls - mode
  
  /**
   * Creates a new ExampleSubsystem.
   */
  public ShootClimbSubsystem() {
    mTalonShoot1 = new PWMTalonFX(Constants.kMotorPWMShoot1);
    mTest = false;
    mColor1 = new DigitalOutput(7);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (mTest) {
      mTalonShoot1.setSpeed(0.5);
      mColor1.set(true);
    } else {
      mTalonShoot1.setSpeed(0);
      mColor1.set(false);
    }
  }

    /**
   * For Intaking: Enables the Sequencer Index Mode.
   */
  public void on() {
    mTest = true;
  }
  /**
   * For Intaking: Enables the Sequencer Index Mode.
   */
  public void off() {
    mTest = false;
  }

}
