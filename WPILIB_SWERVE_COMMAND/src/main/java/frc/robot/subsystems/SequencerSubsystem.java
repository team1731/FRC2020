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
//import edu.wpi.first.wpilibj.Timer;

public class SequencerSubsystem extends SubsystemBase {

  private final PWMTalonFX mTalonSeq;
  private DigitalInput mLowSensor;
  private DigitalInput mMidSensor;
  private DigitalInput mHighSensor;
  //private Timer mTimer;
  //private double elapsed;
  //private boolean startDelay;
  //private boolean mLowSensorCur;
  private boolean mLastLowHasBall; // does robot want to index balls - mode
  private boolean mLastHighHasBall; // does robot want to index balls - mode
  private int mPowerCellCount;

  /**
   * Creates a new SequencerSubsystem.
   */
  public SequencerSubsystem() {
    mTalonSeq = new PWMTalonFX(OpConstants.kMotorPWMSeq);
    mLowSensor = new DigitalInput(OpConstants.kLowSequencer);
    mMidSensor = new DigitalInput(OpConstants.kMidSequencer);
    mHighSensor = new DigitalInput(OpConstants.kHighSequencer);
    //mTimer = new Timer();
    //mTimer.start();
    //startDelay = false;
    //mLowSensorCur = mLowSensor.get();
    mLastLowHasBall = lowSensorHasBall();
    mLastHighHasBall = highSensorHasBall();
    mPowerCellCount = 0;
  }

  public void setPowerCellCount(int numBalls){
    mPowerCellCount = numBalls;
  }

  @Override
  public void periodic() {
    // Decrementing ball count
    if (lowSensorHasBall()) {
      if (!mLastLowHasBall) {
        if (mPowerCellCount <= OpConstants.kMaxPowerCells) {
          mPowerCellCount++;
        }
      }
    }
    // Decrementing ball count
    if (!highSensorHasBall()) {
      if (mLastHighHasBall) {
        if (mPowerCellCount > 0) {
          mPowerCellCount--;
        }
      }
    }

    mLastLowHasBall = lowSensorHasBall();
    mLastHighHasBall = highSensorHasBall();
  }
  
  /**
   * For Shooting: Enables the Sequencer by turning on motor.
   */
  public void forward(boolean shooting) {
    if(shooting){
      mTalonSeq.setSpeed(OpConstants.kMotorSeqFwdShootSpeed);
    }
    else{
      mTalonSeq.setSpeed(OpConstants.kMotorSeqFwdIntakeSpeed);
    }
    //mPowerCellCount = 0;
  }

  /**
   * For Ejecting balls: Reverses the motor.
   */
  public void reverse() {
    mTalonSeq.setSpeed(OpConstants.kMotorSeqRevShootSpeed);
    mPowerCellCount = 0;
  }

  /**
   * Enables the intake by retracting solenoid & turning off motor.
   */
  public void stop() {
    mTalonSeq.setSpeed(0);
    mTalonSeq.stopMotor();
  }

  public boolean lowSensorHasBall() {
    return !mLowSensor.get();
  }

  public boolean midSensorHasBall() {
    return !mMidSensor.get();
  }

  public boolean highSensorHasBall() {
    return !mHighSensor.get();
  }

  public boolean getMaxPowerCells() {
    return(mPowerCellCount >= OpConstants.kMaxPowerCells);
  }

  public int getPowerCellCount() {
    return(mPowerCellCount);
  }

  /*
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    mLowSensorCur = mLowSensor.get();
    if (startDelay) {
      if (mTimer.get() - elapsed > OpConstants.kSeqIntakeDelay) {
        mTalonSeq.setSpeed(0);
        startDelay = false;
      }
    }
  }

  public void addPowerCell() {
    if (mLowSensorCur) {
        startDelay = true;
        elapsed = mTimer.get();
        // incr count at end of intaking powercell
        if (!mLowSensorLast) {
          mPowerCellCount++;
        }
    } else {
        if (mPowerCellCount < OpConstants.kMaxPowerCells) {
            mTalonSeq.setSpeed(OpConstants.kMotorSeqFwdIntakeSpeed);
            // incr count at beginning of intaking powercell
            //if (mLowSensorLast) {
            //    mPowerCellCount++;
            //}
        }
    }
    mLowSensorLast = mLowSensorCur;
  }
  */

}
