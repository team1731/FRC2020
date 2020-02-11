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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * 1731 the Color Wheel rotates (3-5 times) & matches colors
 * 
 * @see ColorWheelSubsystem.java
 */
public class ColorWheelSubsystem extends SubsystemBase {

  private final PWMTalonFX mTalonFX;
  // sensors used for ColorWheel
  private final I2C.Port i2cPort;
  private final ColorSensorV3 mColorSensor;
  private final ColorMatch mColorMatcher;

  private int colorCount;
  private int colorSample;
  private int wheelCount;
  private double matchConfidence;
  private WheelState mWheelState;

  public static final int[] colorSeq = {
      Constants.kWheelRed,
      Constants.kWheelGreen,
      Constants.kWheelBlue,
      Constants.kWheelYellow,
      Constants.kWheelRed,
      Constants.kWheelGreen
  };

  public enum WheelState {
    IDLE,       // doing nothing ... waiting
    PREPARE,    // verify we are reading the color wheel, i.e. motor is close enough to engage wheel
    OVERRIDE,   // don't care if PREPARE is satisfied, engage anyway
    ENGAGE,     // set solenoid (if any)
    START,      // 1) read color 2) determine next color set as sample 3) start motor
    COUNT,      // 1) read color 2) incr count if color == sample 3) check if count is >= 6, 4) signal led strip 
    NEXT,       // 1) read color 2) hold here until color != sample 
    STOP,       // stop motor
    DISENGAGE   // unset solenoid 
  }

  public enum WheelMode {
    IDLE,       // doing nothing 
    ROTATE,     // rotate 3-5 times
    MATCH       // match color and stay for 5 seconds
  }
  /**
   * Creates a new ExampleSubsystem.
   */
  public ColorWheelSubsystem() {
    mTalonFX = new PWMTalonFX(Constants.kColorWheelTalonFX);
    i2cPort = I2C.Port.kOnboard;
    mColorSensor = new ColorSensorV3(i2cPort);
    mColorMatcher = new ColorMatch();
    mWheelState = WheelState.IDLE;

    mColorMatcher.addColorMatch(Constants.kBlueTarget);
    mColorMatcher.addColorMatch(Constants.kGreenTarget);
    mColorMatcher.addColorMatch(Constants.kRedTarget);
    mColorMatcher.addColorMatch(Constants.kYellowTarget);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void init() {
    // initialization stuff
  }

  private void handleColorWheel(WheelMode mode) {
    int zColor;
    WheelState newState = mWheelState; // set new state to current state; idea is to look for changes
    switch (mWheelState) {
        case IDLE:
            break;
        case PREPARE: // verify we are reading the color wheel, i.e. motor is close enough to engage wheel
            if (checkSensor()) {
                newState = WheelState.ENGAGE;
             } else {
                 // SIGNAL led strip that don't see color wheel
             }
            colorSample = Constants.kWheelUnknown;    
            break;
        case OVERRIDE:
            // check override button if any is used, otherwise this is of no consequence
            break;
        case ENGAGE:
            // SET Solenoid
            newState = WheelState.START;
            break;
        case START: // 1) read color 2) determine next color set as sample 3) start motor
            colorCount = 0;
            zColor = getMatch();
            if (zColor > Constants.kWheelUnknown) {
                switch (mode) {
                    case ROTATE:
                        colorSample = colorSeq[zColor + 1]; // our sample color is the next one after our match
                        wheelCount = Constants.kWheelCountRotate;
                        mTalonFX.setSpeed(Constants.kWheelRotateSpeed);
                        break;
                    case MATCH:
                        colorSample = getGameColor();   // GET color to Match & determine direction
                        wheelCount = Constants.kWheelCountMatch;
                        mTalonFX.setSpeed(Constants.kWheelMatchFwdSpeed);
                        break;
                    default:
                        break;
                }
                if (colorSample > Constants.kWheelUnknown) {
                    newState = WheelState.COUNT;  // only move on if a valid colorSample
                    // SIGNAL led strip that can't get game color data
                }
            }
            break;
        case COUNT: // 1) read color 2) incr count if color == sample 3) check if count is >= 6, 4) signal led strip
            zColor = getMatch();
            if (zColor == colorSample) {
                if (++colorCount >= wheelCount) {    // increment count and check if reached 3 or more rotations
                    newState = WheelState.STOP;
                    // SIGNAL led strip that we are successful
                } else {
                    newState = WheelState.NEXT;
                }
            }
            break;
        case NEXT: // 1) read color 2) hold here until color doesn't equal our sample
            zColor = getMatch();
            if (zColor != colorSample) {
                newState = WheelState.COUNT; // go back to counting
            }
            break;
        case STOP:  // stop motor
            mTalonFX.setSpeed(0);
            newState = WheelState.COUNT;
            break;
        case DISENGAGE:
            // UNSET solonoid
            newState = WheelState.IDLE;
            //mWantedState = WantedState.IDLE;  // return to IDLEing
            break;
        default:
            newState = WheelState.IDLE;
            //mWantedState = WantedState.IDLE;  // return to IDLEing
    }

    if (newState != mWheelState) {
        // System.out.println("ColorWheel state " + mSystemState + " to " + newState);
        mWheelState = newState;
    }
  }

  public void handleRotating() {
    mWheelState = WheelState.PREPARE;
    handleColorWheel(WheelMode.ROTATE);
  }

  public void handleMatching() {
    mWheelState = WheelState.PREPARE;
    handleColorWheel(WheelMode.MATCH);
  }


  private Color getColor() {
    //Color detectedColor = mColorSensor.getColor();
    /* The sensor returns a raw IR value of the infrared light detected. */
    //double IR = mColorSensor.getIR();
    return mColorSensor.getColor();
  }

  private int getMatch() {
    Color detectedColor = mColorSensor.getColor();

    /* Run the color match algorithm on our detected color */
    int colorId;
    ColorMatchResult match = mColorMatcher.matchClosestColor(detectedColor);
    matchConfidence = match.confidence;

    if (match.color == Constants.kBlueTarget) {
        colorId = Constants.kWheelBlue;
    } else if (match.color == Constants.kRedTarget) {
        colorId = Constants.kWheelRed;
    } else if (match.color == Constants.kGreenTarget) {
        colorId = Constants.kWheelGreen;
    } else if (match.color == Constants.kYellowTarget) {
        colorId = Constants.kWheelYellow;
    } else {
        colorId = Constants.kWheelUnknown;
    }

    return colorId;
  }

  private int getGameColor() {
    String gameData;
    int color = Constants.kWheelUnknown;
    gameData = DriverStation.getInstance().getGameSpecificMessage();
    if(gameData.length() > 0)
    {
        switch (gameData.charAt(0))
        {
            case 'B' : color = Constants.kWheelBlue; break; //Blue case code
            case 'G' : color = Constants.kWheelGreen; break; //Green case code
            case 'R' : color = Constants.kWheelRed; break; //Red case code
            case 'Y' : color = Constants.kWheelYellow; break; //Yellow case code
            default :  color = Constants.kWheelUnknown; break; //This is corrupt data
        }
    } else {
      color = Constants.kWheelUnknown; //Code for no data received yet
    }
    return color;
  }

  public boolean checkSensor() {
    //return ((mIRSensor1.getAverageValue() > 300) && (mIRSensor2.getAverageValue() > 300)); 
     return true;
  }

  public void outputToSmartDashboard() {
    // SmartDashboard.putNumber("IRSensor1", mIRSensor1.getAverageValue());
    // SmartDashboard.putNumber("IRSensor2", mIRSensor2.getAverageValue());
     /*
      * SmartDashboard.putNumber("ColorWheelmWantedState", mWantedState);
      * SmartDashboard.putBoolean("ElevRevSw",
      * mTalon.getSensorCollection().isRevLimitSwitchClosed());
      */
     Color detectedColor = getColor();
     SmartDashboard.putNumber("Red", detectedColor.red);
     SmartDashboard.putNumber("Green", detectedColor.green);
     SmartDashboard.putNumber("Blue", detectedColor.blue);
     // The sensor returns a raw IR value of the infrared light detected.
     SmartDashboard.putNumber("IR", mColorSensor.getIR());
   
     SmartDashboard.putNumber("Confidence", matchConfidence);
     SmartDashboard.putNumber("Detected Color", getMatch());
 }

}
