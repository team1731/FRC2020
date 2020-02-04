package org.usfirst.frc.team1731.robot.subsystems;

import org.usfirst.frc.team1731.robot.Constants;
import org.usfirst.frc.team1731.robot.loops.Loop;
import org.usfirst.frc.team1731.robot.loops.Looper;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.PWMTalonFX;

import edu.wpi.first.wpilibj.DriverStation;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
//import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
//import edu.wpi.first.wpilibj.VictorSP;

import org.usfirst.frc.team1731.robot.subsystems.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


//import edu.wpi.first.wpilibj.DoubleSolenoid;
//import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

/**
 * 1731 the shooter shootes the balls and climbs
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


    //private final PWMTalonFX mTalonShoot1;
    //private final PWMTalonFX mTalonShoot2;
    private final TalonFX mTalonShoot1;
    private final TalonFX mTalonShoot2;
    private SystemState mSystemState = SystemState.IDLE;
    private WantedState mWantedState = WantedState.IDLE;
    //private DoubleSolenoid IntakeHood;

    private double mCurrentStateStartTime;
    private boolean mStateChanged = false;

    private Shooter() {
        //mTalonShoot1 = new PWMTalonFX(Constants.kMotorPWMShoot1);
        //mTalonShoot2 = new PWMTalonFX(Constants.kMotorPWMShoot2);
        mTalonShoot1 = new TalonFX(Constants.kMotorCANShoot1);
        mTalonShoot2 = new TalonFX(Constants.kMotorCANShoot2);
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
        SHOOTING,
        CLIMB_ENGAGING,
        CLIMB_EXTENDING,
        CLIMB_ALIGNING,
        CLIMB_RETRACTING
    }

    public enum WantedState {
        IDLE,
        SHOOT,
        CLIMB_ENGAGE,
        CLIMB_EXTEND,
        CLIMB_ALIGN,
        CLIMB_RETRACT
    }

    private final Loop mLoop = new Loop() {
        @Override
        public void onStart(final double timestamp) {
            stop();
            synchronized (Shooter.this) {
                mSystemState = SystemState.IDLE;
                mStateChanged = true;
                mCurrentStateStartTime = timestamp;
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
                    case CLIMB_ENGAGING:
                        newState = handleClimbEngaging();
                        break;
                    case CLIMB_EXTENDING:
                        newState = handleClimbExtending();
                        break;
                    case CLIMB_ALIGNING:
                        newState = handleClimbAligning();
                        break;
                    case CLIMB_RETRACTING:
                        newState = handleClimbRetracting();
                        break;
                    default:
                        newState = SystemState.IDLE;
                }

                if (newState != mSystemState) {
                    // System.out.println("Shooter state " + mSystemState + " to " + newState);
                    mSystemState = newState;
                    mCurrentStateStartTime = timestamp;
                    // DriverStation.reportWarning("Shooter SystemState: " + mSystemState, false);
                    mStateChanged = true;
                } else {
                    mStateChanged = false;
                }
            }
        }

        private SystemState handleShooting() {
            if (mStateChanged) {
                //mTalonShoot1.setSpeed(Constants.kMotorShootSpeed);
                //mTalonShoot2.setSpeed(Constants.kMotorShootSpeed);
                mTalonShoot1.set(ControlMode.PercentOutput,Constants.kMotorShootSpeed1);
                mTalonShoot2.set(ControlMode.PercentOutput,Constants.kMotorShootSpeed2);
            }
            return defaultStateTransfer();
        }

        private SystemState handleClimbEngaging() {
            if (mStateChanged) {
                // IntakeHood.set(Value.kForward);             
            }
            return defaultStateTransfer();
        }

        private SystemState handleClimbExtending() {
            if (mStateChanged) {
                // IntakeHood.set(Value.kForward);
            }
            return defaultStateTransfer();
        }

        private SystemState handleClimbAligning() {
            if (mStateChanged) {
                // IntakeHood.set(Value.kForward);
            }
            return defaultStateTransfer();
        }

        private SystemState handleClimbRetracting() {
            if (mStateChanged) {
                // IntakeHood.set(Value.kForward);
                mTalonShoot1.set(ControlMode.PercentOutput,Constants.kMotorClimbPercent);
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
            case SHOOT:
                return SystemState.SHOOTING;
            case CLIMB_ENGAGE:
                return SystemState.CLIMB_ENGAGING;
            case CLIMB_EXTEND:
                return SystemState.CLIMB_EXTENDING;
            case CLIMB_ALIGN:
                return SystemState.CLIMB_ALIGNING;
            case CLIMB_RETRACT:
                return SystemState.CLIMB_RETRACTING;
            default:
                return SystemState.IDLE;
        }
    }

    private SystemState handleIdle() {
        // setOpenLoop(0.0f);
        // if motor is not off, turn motor off
        if (mStateChanged) {
            //mTalonShoot1.setSpeed(0);
            //mTalonShoot2.setSpeed(0);
            mTalonShoot1.set(ControlMode.PercentOutput,0);
            mTalonShoot2.set(ControlMode.PercentOutput,0);
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
         * mTalon.getSensorCollection().getQuadraturePosition());
         * SmartDashboard.putBoolean("ElevRevSw",
         * mTalon.getSensorCollection().isRevLimitSwitchClosed());
         */
        SmartDashboard.putString("ShootWantState", mWantedState.name());
        SmartDashboard.putString("ShootSysState", mSystemState.name());
        //SmartDashboard.putBoolean("ShootTalon1Alive", mTalonShoot1.);
        //SmartDashboard.putBoolean("ShootTalon2Alive", mTalonShoot2.);
        SmartDashboard.putNumber("ShootSpd1", mTalonShoot1.getMotorOutputPercent());
        SmartDashboard.putNumber("ShootSpd2", mTalonShoot2.getMotorOutputPercent());
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

    public synchronized SystemState getSystemState() {
        return mSystemState;
    }

}
    
