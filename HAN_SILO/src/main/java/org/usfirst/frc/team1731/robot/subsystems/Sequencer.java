package org.usfirst.frc.team1731.robot.subsystems;
import java.util.Arrays;

import edu.wpi.first.wpilibj.DigitalInput;

import org.usfirst.frc.team1731.lib.util.MovingAverage;
import org.usfirst.frc.team1731.lib.util.Util;
import org.usfirst.frc.team1731.lib.util.drivers.TalonSRXFactory;
import org.usfirst.frc.team1731.robot.Constants;
import org.usfirst.frc.team1731.robot.loops.Loop;
import org.usfirst.frc.team1731.robot.loops.Looper;
import org.usfirst.frc.team1731.robot.subsystems.Elevator.SystemState;
import org.usfirst.frc.team1731.robot.subsystems.Elevator.WantedState;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DriverStation;

//import com.ctre.CANTalon;
//Getting rid of all solenoid for testing!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
//import edu.wpi.first.wpilibj.VictorSP;

import org.usfirst.frc.team1731.robot.subsystems.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.AnalogInput;

/**
 * 1731 the intake picks up cubes and ejects them
 * 
 * @see Subsystem.java
 */
@SuppressWarnings("unused")
public class Sequencer extends Subsystem {
    private static Sequencer sInstance = null;

    public static Sequencer getInstance() {
        if (sInstance == null) {
            sInstance = new Sequencer();
        }
        return sInstance;
    }

    private final TalonFX mTalonFX;
    private DigitalInput sequencerSensorLow = new DigitalInput(Constants.kLowSequencer);
    private DigitalInput sequencerSensorHigh = new DigitalInput(Constants.kHighSequencer);


    private Sequencer() {
        mTalonFX = new TalonFX(Constants.kSequencerVictor);
        //mIRSensor1 = new AnalogInput(1);
        //mIRSensor2 = new AnalogInput(4);
    }

    public boolean checkSystem() {

        return true;
    }

    public void setIdle() {
        // TODO Auto-generated method stub

    }

    public enum SystemState {
        IDLE, // stop all motors
        INTAKING, EJECT, SHOOTING,
    }

    public enum WantedState {
        IDLE, INTAKING, // moving
        EJECT, SHOOTING, 
    }

    private SystemState mSystemState = SystemState.IDLE;
    private WantedState mWantedState = WantedState.IDLE;

    //DoubleSolenoid IntakeHood = new DoubleSolenoid(Constants.kIntakeHoodSolenoid1, Constants.kIntakeHoodSolenoid2);

    private double mCurrentStateStartTime;
    // private double mWantedPosition = 0;
    private boolean mStateChanged = false;

    private final Loop mLoop = new Loop() {
        @Override
        public void onStart(final double timestamp) {
            stop();
            synchronized (Sequencer.this) {
                mSystemState = SystemState.IDLE;
                mStateChanged = true;
                // mWantedPosition = 0;
                mCurrentStateStartTime = timestamp;
                // DriverStation.reportError("Elevator SystemState: " + mSystemState, false);
            }
        }

        @Override
        public void onLoop(final double timestamp) {

            synchronized (Sequencer.this) {
                SystemState newState;
                switch (mSystemState) {
                case IDLE:
                    newState = handleIdle();
                    break;
                case INTAKING:
                    newState = handleIntaking();
                    break;
                case EJECT:
                    newState = handleEjecting();
                    break;
                case SHOOTING: 
                    newState= handleShooting();
                    break;
                default:
                    newState = SystemState.IDLE;
                }

                if (newState != mSystemState) {
                    // System.out.println("Elevator state " + mSystemState + " to " + newState);
                    mSystemState = newState;
                    mCurrentStateStartTime = timestamp;
                    // DriverStation.reportWarning("Intake SystemState: " + mSystemState, false);
                    mStateChanged = true;
                } else {
                    mStateChanged = false;
                }
            }
        }

        private SystemState handleIntaking() {
            if (mStateChanged) {
                if (!sequencerSensorLow.get()){
                    mTalonFX.set(ControlMode.PercentOutput, 1);
                } 
                if (sequencerSensorLow.get()) {
                    mTalonFX.set(ControlMode.PercentOutput, 0);
                }
                if (!sequencerSensorHigh.get()){
                    mTalonFX.set(ControlMode.PercentOutput, 0);
                }
            }
            return defaultStateTransfer();
        }

        private SystemState handleEjecting() {
            if (mStateChanged) {
                mTalonFX.set(ControlMode.PercentOutput, -1);
            }
            return defaultStateTransfer();
        }

        private SystemState handleShooting() {
            if (mStateChanged) {
                mTalonFX.set(ControlMode.PercentOutput, 1);
            }
            return defaultStateTransfer();
        }

        @Override
        public void onStop(final double timestamp) {
            stop();
        }
    };

    private SystemState defaultStateTransfer() {
        switch (mWantedState) {
        case INTAKING:
            return SystemState.INTAKING;
        case EJECT:
            return SystemState.EJECT;
        case SHOOTING:
            return SystemState.SHOOTING;

        default:
            return SystemState.IDLE;
        }
    }

    private SystemState handleIdle() {
        // if motor is not off, turn motor off
        if (mStateChanged) {
            mTalonFX.set(ControlMode.PercentOutput, 0);
        }
        return defaultStateTransfer();
    }

    public synchronized void setWantedState(final WantedState state) {
        if (state != mWantedState) {
            mWantedState = state;
            // DriverStation.reportError("Intake WantedState: " + mWantedState, false);
        }
    }

    @Override
    public void outputToSmartDashboard() {
       // SmartDashboard.putNumber("IRSensor1", mIRSensor1.getAverageValue());
       // SmartDashboard.putNumber("IRSensor2", mIRSensor2.getAverageValue());
        /*
         * SmartDashboard.putNumber("ElevWantPos", mWantedState);
         * SmartDashboard.putNumber("ElevCurPos", mTalon.getSelectedSensorPosition(0));
         * SmartDashboard.putNumber("ElevQuadPos",
         * mTalon.getSensorCollection().getQuadraturePosition());
         * SmartDashboard.putBoolean("ElevRevSw",
         * mTalon.getSensorCollection().isRevLimitSwitchClosed());
         */
    }

    @Override
    public void stop() {
        // mTalonFX.set(0);
        setWantedState(WantedState.IDLE);
    }

    @Override
    public void zeroSensors() {
    }

    @Override
    public void registerEnabledLoops(final Looper in) {
        in.register(mLoop);
    }
    
    //public boolean gotCube() {
    //	 return ((mIRSensor1.getAverageValue() > 300) && (mIRSensor2.getAverageValue() > 300)); 
    //}

    public synchronized SystemState getSystemState() {
        return mSystemState;
    }

}
    
