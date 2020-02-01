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
//disabled solenoids for testing
//import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
//import edu.wpi.first.wpilibj.VictorSP;

//import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import org.usfirst.frc.team1731.robot.subsystems.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.AnalogInput;
//import edu.wpi.first.wpilibj.DoubleSolenoid;

/**
 * 1731 the intake picks up cubes and ejects them
 * 
 * @see Subsystem.java
 */
@SuppressWarnings("unused")
public class Climber extends Subsystem {
    private static Climber sInstance = null;

    public static Climber getInstance() {
        if (sInstance == null) {
            sInstance = new Climber();
        }
        return sInstance;
    }


    //private final PWMTalonFX mTalonFX;
    // sensors used for old intake. New intake doesn't need them.
   // private final AnalogInput mIRSensor1;
    //private final AnalogInput mIRSensor2;

    private Climber() {
        //did randome numbor for testing. Line 55 is what we want. ignore 5.
        //mTalonFX = new PWMTalonFX(5);
        //mTalonFX = new PWMTalonFX(Constants.kShooterVictor);
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
        EXTENDING, RETRACTING ,
    }

    public enum WantedState {
        IDLE, EXTENDING, // moving
        RETRACTING ,
    }

    private SystemState mSystemState = SystemState.IDLE;
    private WantedState mWantedState = WantedState.IDLE;

    //DoubleSolenoid ShooterGearShift = new DoubleSolenoid(Constants.kShooterToClimberSolenoid1, Constants.kShooterToClimberSolenoid2);
    //DoubleSolenoid ClimberSolenoid = new DoubleSolenoid(Constants.kClimberSolenoid1, Constants.kClimberSolenoid2);

    private double mCurrentStateStartTime;
    // private double mWantedPosition = 0;
    private boolean mStateChanged = false;

    private final Loop mLoop = new Loop() {
        @Override
        public void onStart(final double timestamp) {
            stop();
            synchronized (Climber.this) {
                mSystemState = SystemState.IDLE;
                mStateChanged = true;
                // mWantedPosition = 0;
                mCurrentStateStartTime = timestamp;
                // DriverStation.reportError("Elevator SystemState: " + mSystemState, false);
            }
        }

        @Override
        public void onLoop(final double timestamp) {

            synchronized (Climber.this) {
                SystemState newState;
                switch (mSystemState) {
                case IDLE:
                    newState = handleIdle();
                    break;
                case EXTENDING:
                    newState = handleExtending();
                    break;
                case RETRACTING :
                    newState = handleRetracting();
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

        private SystemState handleExtending() {
            if (mStateChanged) {
                //ShooterGearShift.set(Value.kForward);
                //ClimberSolenoid.set(Value.kForward);
                //mTalonFX.setSpeed(0.2);
            }
            return defaultStateTransfer();
        }

        private SystemState handleRetracting() {
            if (mStateChanged) {
                //ShooterGearShift.set(Value.kForward);
                //ClimberSolenoid.set(Value.kOff);
                //mTalonFX.setSpeed(-.2);
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
        case EXTENDING:
            return SystemState.EXTENDING;
        case RETRACTING :
            return SystemState.RETRACTING ;

        default:
            return SystemState.IDLE;
        }
    }

    private SystemState handleIdle() {
        // setOpenLoop(0.0f);
        // if motor is not off, turn motor off
        if (mStateChanged) {
            //if (mTalonFX.getSpeed() > 0) {
            //mTalonFX.setSpeed(0);
            //ShooterGearShift.set(Value.kReverse);
           // } 
            /*
            if (ShooterGearShift.getValue() == kForward) {
            ShooterGearShift.set(Value.kReverse);
            }
            */
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
    
