package org.usfirst.frc.team1731.robot.subsystems;
//import java.util.Arrays;

//import org.usfirst.frc.team1731.lib.util.Util;
//import org.usfirst.frc.team1731.lib.util.drivers.TalonSRXFactory;
import org.usfirst.frc.team1731.robot.Constants;
import org.usfirst.frc.team1731.robot.loops.Loop;
import org.usfirst.frc.team1731.robot.loops.Looper;

//import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.PWMTalonFX;

//import edu.wpi.first.wpilibj.DriverStation;

//import com.ctre.CANTalon;
//import edu.wpi.first.wpilibj.Solenoid;
//import edu.wpi.first.wpilibj.Timer;
//import edu.wpi.first.wpilibj.VictorSP;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;

import org.usfirst.frc.team1731.robot.subsystems.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * 1731 the Color Wheel rotates (3-5 times) & matches colors
 * 
 * @see Subsystem.java
 */
@SuppressWarnings("unused")
public class ColorWheel extends Subsystem {
    private static ColorWheel sInstance = null;

    public static ColorWheel getInstance() {
        if (sInstance == null) {
            sInstance = new ColorWheel();
        }
        return sInstance;
    }

    private final PWMTalonFX mTalonFX;
    // sensors used for ColorWheel.
    private final I2C.Port i2cPort;
    private final ColorSensorV3 mColorSensor;
    private final ColorMatch mColorMatcher;
  
    private ColorWheel() {
        mTalonFX = new PWMTalonFX(Constants.kColorWheelTalonFX);
        i2cPort = I2C.Port.kOnboard;
        mColorSensor = new ColorSensorV3(i2cPort);
        mColorMatcher = new ColorMatch();

        mColorMatcher.addColorMatch(Constants.kBlueTarget);
        mColorMatcher.addColorMatch(Constants.kGreenTarget);
        mColorMatcher.addColorMatch(Constants.kRedTarget);
        mColorMatcher.addColorMatch(Constants.kYellowTarget);    
    }

    public boolean checkSystem() {
        return checkSensor();
    }

    public void setIdle() {
        // TODO Auto-generated method stub
    }

    public enum SystemState {
        IDLE, // stop all motors
        ROTATING,
        MATCHING,
    }

    public enum WantedState {
        IDLE,
        ROTATE, // moving
        MATCH,
    }

    private SystemState mSystemState = SystemState.IDLE;
    private WantedState mWantedState = WantedState.IDLE;

    //Solenoid sColorWheel = new Solenoid(Constants.kColorWheelSolenoid);

    private double mCurrentStateStartTime;
    // private double mWantedPosition = 0;
    private boolean mStateChanged = false;

    private final Loop mLoop = new Loop() {
        @Override
        public void onStart(final double timestamp) {
            stop();
            synchronized (ColorWheel.this) {
                mSystemState = SystemState.IDLE;
                mStateChanged = true;
                mCurrentStateStartTime = timestamp;
                // DriverStation.reportError("ColorWheel SystemState: " + mSystemState, false);
            }
        }

        @Override
        public void onLoop(final double timestamp) {

            synchronized (ColorWheel.this) {
                SystemState newState;
                switch (mSystemState) {
                    case IDLE:
                        newState = handleIdle();
                        break;
                    case ROTATING:
                        newState = handleRotating();
                        break;
                    case MATCHING:
                        newState = handleMatching();
                        break;
                    default:
                        newState = SystemState.IDLE;
                }

                if (newState != mSystemState) {
                    // System.out.println("ColorWheel state " + mSystemState + " to " + newState);
                    mSystemState = newState;
                    mCurrentStateStartTime = timestamp;
                    // DriverStation.reportWarning("ColorWheel SystemState: " + mSystemState, false);
                    mStateChanged = true;
                } else {
                    mStateChanged = false;
                }
            }
        }

        private SystemState handleRotating() {
            if (mStateChanged) {
                //ColorWheelHood.set(Value.kForward);
                mTalonFX.setSpeed(.5);
            }
            return defaultStateTransfer();
        }

        private SystemState handleMatching() {
            if (mStateChanged) {
                //ColorWheelHood.set(Value.kForward);
                mTalonFX.setSpeed(-.2);
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
            case ROTATE:
                return SystemState.ROTATING;
            case MATCH:
                return SystemState.MATCHING;

            default:
                return SystemState.IDLE;
        }
    }

    private SystemState handleIdle() {
        // setOpenLoop(0.0f);
        // if motor is not off, turn motor off
        if (mStateChanged) {
            mTalonFX.setSpeed(0);
            //IntakeHood.set(Value.kReverse);
        }
        return defaultStateTransfer();
    }

    private Color getColor() {
        //Color detectedColor = mColorSensor.getColor();
        /* The sensor returns a raw IR value of the infrared light detected. */
        //double IR = mColorSensor.getIR();
        return mColorSensor.getColor();
    }

    private String getMatch() {
        Color detectedColor = mColorSensor.getColor();

        /* Run the color match algorithm on our detected color */
        String colorString;
        ColorMatchResult match = mColorMatcher.matchClosestColor(detectedColor);
    
        if (match.color == Constants.kBlueTarget) {
          colorString = "Blue";
        } else if (match.color == Constants.kRedTarget) {
          colorString = "Red";
        } else if (match.color == Constants.kGreenTarget) {
          colorString = "Green";
        } else if (match.color == Constants.kYellowTarget) {
          colorString = "Yellow";
        } else {
          colorString = "Unknown";
        }

        return colorString;
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
        Color detectedColor = getColor();
        SmartDashboard.putNumber("Red", detectedColor.red);
        SmartDashboard.putNumber("Green", detectedColor.green);
        SmartDashboard.putNumber("Blue", detectedColor.blue);
        // The sensor returns a raw IR value of the infrared light detected.
        SmartDashboard.putNumber("IR", mColorSensor.getIR());
      
        //SmartDashboard.putNumber("Confidence", match.confidence);
        SmartDashboard.putString("Detected Color", getMatch());
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
    
    public boolean checkSensor() {
    	 //return ((mIRSensor1.getAverageValue() > 300) && (mIRSensor2.getAverageValue() > 300)); 
        return true;
    }

    public synchronized SystemState getSystemState() {
        return mSystemState;
    }

}
    
