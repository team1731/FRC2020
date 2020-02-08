/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.PWMTalonFX;
import edu.wpi.first.wpilibj.DigitalInput;


public class SequencerSubsystem extends SubsystemBase {

  private final PWMTalonFX mTalonSeq;
  private DigitalInput mLowSensor;
  private boolean mLowSensorLast;
  private boolean mIndexing; // does robot want to index balls - mode
  private int mPowerCellCount;

  /**
   * Creates a new SequencerSubsystem.
   */
  public SequencerSubsystem() {
    mTalonSeq = new PWMTalonFX(Constants.kMotorPWMSeq);
    mLowSensor = new DigitalInput(Constants.kLowSequencer);
    mIndexing = false;
    mLowSensorLast = mLowSensor.get();
    mPowerCellCount = 0;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (mIndexing) {
      boolean sensor = mLowSensor.get();
      if (sensor) {
          mTalonSeq.setSpeed(0);
      } else {
          if (mPowerCellCount < 5) {
              mTalonSeq.setSpeed(Constants.kMotorSeqFwdSpeed);
              if (mLowSensorLast) {
                  mPowerCellCount++;
              }
          }
      }
      mLowSensorLast = sensor;
    }
  }

  /**
   * For Intaking: Enables the Sequencer Index Mode.
   */
  public void index() {
    mIndexing = true;
    mPowerCellCount = 0;
  }

  public void addBall() {
    if (mPowerCellCount < 5) {
      mTalonSeq.setSpeed(Constants.kMotorSeqFwdSpeed);
      if (mLowSensorLast) {
          mPowerCellCount++;
      }
    }
    mLowSensorLast = mLowSensor.get();
  }

  /**
   * For Shooting: Enables the Sequencer by turning on motor.
   */
  public void forward() {
    mTalonSeq.setSpeed(Constants.kMotorSeqFwdSpeed);
    mIndexing = false;
    mPowerCellCount = 0;
  }

  /**
   * For Ejecting balls: Reverses the motor.
   */
  public void reverse() {
    mTalonSeq.setSpeed(Constants.kMotorSeqRevSpeed);
    mIndexing = false;
    mPowerCellCount = 0;
  }

  /**
   * Enables the intake by retracting solenoid & turning off motor.
   */
  public void stop() {
    mTalonSeq.setSpeed(0);
    mIndexing = false;
  }

  public boolean getLowSensor() {
    return(mLowSensor.get());
  }

  public boolean getMaxPowerCells() {
    return(mPowerCellCount >= 5);
  }

  public void incrPowerCells() {
    mPowerCellCount += 1;
  }
}
