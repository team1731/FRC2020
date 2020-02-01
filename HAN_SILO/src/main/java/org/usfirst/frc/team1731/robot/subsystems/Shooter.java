package org.usfirst.frc.team1731.robot.subsystems;
import java.util.Arrays;

import org.usfirst.frc.team1731.lib.util.MovingAverage;
import org.usfirst.frc.team1731.lib.util.Util;
import org.usfirst.frc.team1731.lib.util.drivers.TalonSRXFactory;
import org.usfirst.frc.team1731.robot.Constants;
import org.usfirst.frc.team1731.robot.loops.Loop;
import org.usfirst.frc.team1731.robot.loops.Looper;
import org.usfirst.frc.team1731.robot.subsystems.Elevator.SystemState;
import org.usfirst.frc.team1731.robot.subsystems.Elevator.WantedState;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.PWMTalonFX;

import edu.wpi.first.wpilibj.DriverStation;

//import com.ctre.CANTalon;
import edu.wpi.first.wpilibj.Timer;
//import edu.wpi.first.wpilibj.VictorSP;

import org.usfirst.frc.team1731.robot.subsystems.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.AnalogInput;

/**
 * 1731 the shooter shootes the balls
 * 
 * @see Subsystem.java
 */
@SuppressWarnings("unused")
public class Shooter extends Subsystem {
    private static Shooter sInstance = null;

    public static Shooter getInstance() {
        if (sInstance == null) {
            sInstance = new Shooter();
        }
        return sInstance;
    }


    private final PWMTalonFX mTalonFX;

    private Shooter() {
        mTalonFX = new PWMTalonFX(Constants.kShooterVictor);

    }

    public boolean checkSystem() {

        return true;
    }

    public void setIdle() {
        // TODO Auto-generated method stub

    }

    public enum SystemState {
        IDLE, // stop all motors
        SHOOTING,
    }

    public enum WantedState {
        IDLE, SHOOTING,
    }

    private SystemState mSystemState = SystemState.IDLE;
    private WantedState mWantedState = WantedState.IDLE;

    private double mCurrentStateStartTime;
    // private double mWantedPosition = 0;
    private boolean mStateChanged = false;

    private final Loop mLoop = new Loop() {
        @Override
        public void onStart(final double timestamp) {
            stop();
            synchronized (Shooter.this) {
                mSystemState = SystemState.IDLE;
                mStateChanged = true;
                // mWantedPosition = 0;
                mCurrentStateStartTime = timestamp;
                // DriverStation.reportError("Elevator SystemState: " + mSystemState, false);
            }
        }

        @Override
        public void onLoop(final double timestamp) {

            synchronized (Shooter.this) {
                SystemState newState;
                switch (mSystemState) {
                case IDLE:
                    newState = handleIdle();
                    break;
                case SHOOTING:
                    newState = handleShooting();
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

        private SystemState handleShooting() {
            if (mStateChanged) {
                mTalonFX.setSpeed(.2);
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
        case SHOOTING:
            return SystemState.SHOOTING;

        default:
            return SystemState.IDLE;
        }
    }

    private SystemState handleIdle() {
        // setOpenLoop(0.0f);
        // if motor is not off, turn motor off
        if (mStateChanged) {
            mTalonFX.setSpeed(0);
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
    
