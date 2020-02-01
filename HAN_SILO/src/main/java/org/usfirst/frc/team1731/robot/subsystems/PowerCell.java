package org.usfirst.frc.team1731.robot.subsystems;

import org.usfirst.frc.team1731.robot.Constants;
import org.usfirst.frc.team1731.robot.loops.Loop;
import org.usfirst.frc.team1731.robot.loops.Looper;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.PWMTalonFX;

import edu.wpi.first.wpilibj.DriverStation;

//import com.ctre.CANTalon;
//import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
//import edu.wpi.first.wpilibj.VictorSP;

import org.usfirst.frc.team1731.robot.subsystems.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DigitalInput;

//import edu.wpi.first.wpilibj.DoubleSolenoid;
//import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

/**
 * 1731 the intake picks up cubes and ejects them
 * 
 * @see Subsystem.java
 */
@SuppressWarnings("unused")
public class PowerCell extends Subsystem {
    private static PowerCell sInstance = null;

    public static PowerCell getInstance() {
        if (sInstance == null) {
            sInstance = new PowerCell();
        }
        return sInstance;
    }

    private final PWMTalonFX mTalonIntake;
    private final PWMTalonFX mTalonSeq;
    private final PWMTalonFX mTalonShoot1;
    private final PWMTalonFX mTalonShoot2;
    private SystemState mSystemState = SystemState.IDLE;
    private WantedState mWantedState = WantedState.IDLE;
    private DigitalInput mLowSensor;
    private int mPowerCellCount;
    private boolean mLowSensorLast;
    //private DoubleSolenoid IntakeHood;


    private double mCurrentStateStartTime;
    // private double mWantedPosition = 0;
    private boolean mStateChanged = false;

    private PowerCell() {
        mTalonIntake = new PWMTalonFX(Constants.kMotorPWMIntake);
        mTalonSeq = new PWMTalonFX(Constants.kMotorPWMSeq);
        mTalonShoot1 = new PWMTalonFX(Constants.kMotorPWMShoot1);
        mTalonShoot2 = new PWMTalonFX(Constants.kMotorPWMShoot2);
        mLowSensor = new DigitalInput(Constants.kLowSequencer);
        // DoubleSolenoid IntakeHood = new
        // DoubleSolenoid(Constants.kIntakeHoodSolenoid1, Constants.kIntakeHoodSolenoid2);
}

    public boolean checkSystem() {
        return true;
    }

    public void setIdle() {
        // TODO Auto-generated method stub
    }

    public enum SystemState {
        IDLE, // stop all motors
        INTAKING,
        SHOOTING,
        EJECTING
    }

    public enum WantedState {
        IDLE,
        INTAKE,
        SHOOT,
        EJECT
    }

    private final Loop mLoop = new Loop() {
        @Override
        public void onStart(final double timestamp) {
            stop();
            synchronized (PowerCell.this) {
                mSystemState = SystemState.IDLE;
                mStateChanged = true;
                mCurrentStateStartTime = timestamp;
            }
            mPowerCellCount = 0;
            mLowSensorLast = mLowSensor.get();
        }

        @Override
        public void onLoop(final double timestamp) {

            synchronized (PowerCell.this) {
                SystemState newState;
                switch (mSystemState) {
                    case IDLE:
                        newState = handleIdle();
                        break;
                    case INTAKING:
                        newState = handleIntaking();
                        break;
                    case SHOOTING:
                        newState = handleShooting();
                        break;
                    case EJECTING:
                        newState = handleEjecting();
                        break;
                    default:
                        newState = SystemState.IDLE;
                }

                if (newState != mSystemState) {
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
                // IntakeHood.set(Value.kForward);
                mTalonIntake.setSpeed(Constants.kMotorIntakeFwdSpeed);
            } else {
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
            return defaultStateTransfer();
        }

        private SystemState handleShooting() {
            if (mStateChanged) {
                mTalonShoot1.setSpeed(Constants.kMotorShootSpeed);
                mTalonShoot2.setSpeed(Constants.kMotorShootSpeed);
                mPowerCellCount = 0;
            } else {
                if ((mTalonShoot1.getSpeed() >= (Constants.kMotorShootSpeed * Constants.kMotorShootPercent))
                && (mTalonShoot2.getSpeed() >= (Constants.kMotorShootSpeed * Constants.kMotorShootPercent))) {

                //test if shooting motor is up to speed 
                mTalonSeq.setSpeed(Constants.kMotorSeqFwdSpeed);
                }
            }
            return defaultStateTransfer();
        }

        private SystemState handleEjecting() {
            if (mStateChanged) {
                // IntakeHood.set(Value.kForward);
                mTalonIntake.setSpeed(Constants.kMotorIntakeRevSpeed);
                mTalonSeq.setSpeed(Constants.kMotorSeqRevSpeed);
                mTalonShoot1.setSpeed(0);
                mTalonShoot1.setSpeed(0);
                mPowerCellCount = 0;
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
            case INTAKE:
                return SystemState.INTAKING;
            case SHOOT:
                return SystemState.SHOOTING;
            case EJECT:
                return SystemState.EJECTING;
            default:
                return SystemState.IDLE;
        }
    }

    private SystemState handleIdle() {
        // setOpenLoop(0.0f);
        // if motors are not off, turn motors off & retract intake mechanism/roller
        if (mStateChanged) {
            mTalonIntake.setSpeed(0);
            mTalonSeq.setSpeed(0);
            mTalonShoot1.setSpeed(0);
            mTalonShoot2.setSpeed(0);
            // IntakeHood.set(Value.kReverse);
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
        /*
         * SmartDashboard.putNumber("ElevCurPos", mTalon.getSelectedSensorPosition(0));
         * SmartDashboard.putNumber("ElevQuadPos",
         * mTalon.getSensorCollection().getQuadraturePosition());
         * SmartDashboard.putBoolean("ElevRevSw",
         * mTalon.getSensorCollection().isRevLimitSwitchClosed());
         */
        SmartDashboard.putString("IntakeWantState", mWantedState.name());
        SmartDashboard.putString("IntakeSysState", mSystemState.name());
        SmartDashboard.putBoolean("IntakeTalonAlive", mTalonIntake.isAlive());
        SmartDashboard.putBoolean("IntakeLowSensor", mLowSensor.get());
        SmartDashboard.putNumber("PCellCount", mPowerCellCount);
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

    // public boolean gotCube() {
    // return ((mIRSensor1.getAverageValue() > 300) && (mIRSensor2.getAverageValue()
    // > 300));
    // }

    public synchronized SystemState getSystemState() {
        return mSystemState;
    }

}
