/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OpConstants;
import edu.wpi.first.wpilibj.PWMTalonFX;
import edu.wpi.first.wpilibj.DigitalInput;


public class SequencerSubsystem extends SubsystemBase {

  private final PWMTalonFX mTalonSeq;
  private DigitalInput mLowSensor;
  private boolean mLowSensorCur;
  private boolean mLowSensorLast; // does robot want to index balls - mode
  private int mPowerCellCount;

  /**
   * Creates a new SequencerSubsystem.
   */
  public SequencerSubsystem() {
    mTalonSeq = new PWMTalonFX(OpConstants.kMotorPWMSeq);
    mLowSensor = new DigitalInput(OpConstants.kLowSequencer);
    mLowSensorCur = mLowSensor.get();
    mLowSensorLast = mLowSensorCur;
    mPowerCellCount = 0;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    mLowSensorCur = mLowSensor.get();
  }

  public void addPowerCell() {
    if (mLowSensorCur) {
        mTalonSeq.setSpeed(0);
        // incr count at end of intaking powercell
        if (!mLowSensorLast) {
          mPowerCellCount++;
        }
    } else {
        if (mPowerCellCount < OpConstants.kMaxPowerCells) {
            mTalonSeq.setSpeed(OpConstants.kMotorSeqFwdSpeed);
            // incr count at beginning of intaking powercell
            //if (mLowSensorLast) {
            //    mPowerCellCount++;
            //}
        }
    }
    mLowSensorLast = mLowSensorCur;
  }

  /**
   * For Shooting: Enables the Sequencer by turning on motor.
   */
  public void forward() {
    mTalonSeq.setSpeed(OpConstants.kMotorSeqFwdSpeed);
    mPowerCellCount = 0;
  }

  /**
   * For Ejecting balls: Reverses the motor.
   */
  public void reverse() {
    mTalonSeq.setSpeed(OpConstants.kMotorSeqRevSpeed);
    mPowerCellCount = 0;
  }

  /**
   * Enables the intake by retracting solenoid & turning off motor.
   */
  public void stop() {
    mTalonSeq.setSpeed(0);
  }

  public boolean getLowSensor() {
    return(mLowSensorCur);
  }

  public boolean getMaxPowerCells() {
    return(mPowerCellCount >= OpConstants.kMaxPowerCells);
  }

  public int getPowerCellCount() {
    return(mPowerCellCount);
  }
}
