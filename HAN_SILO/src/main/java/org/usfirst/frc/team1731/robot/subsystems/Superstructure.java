package org.usfirst.frc.team1731.robot.subsystems;

import java.util.Optional;

import org.usfirst.frc.team1731.lib.util.CircularBuffer;
import org.usfirst.frc.team1731.lib.util.InterpolatingDouble;
import org.usfirst.frc.team1731.lib.util.drivers.RevRoboticsAirPressureSensor;
import org.usfirst.frc.team1731.robot.Constants;
import org.usfirst.frc.team1731.robot.Constants.GRABBER_POSITION;
import org.usfirst.frc.team1731.robot.Constants.ELEVATOR_POSITION;
import org.usfirst.frc.team1731.robot.Robot;
import org.usfirst.frc.team1731.robot.RobotState;
import org.usfirst.frc.team1731.robot.ShooterAimingParameters;
import org.usfirst.frc.team1731.robot.loops.Loop;
import org.usfirst.frc.team1731.robot.loops.Looper;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;


/**
 * The superstructure subsystem is the overarching superclass containing all components of the superstructure: the
 * intake, hopper, feeder, shooter and LEDs. The superstructure subsystem also contains some miscellaneous hardware that
 * is located in the superstructure but isn't part of any other subsystems like the compressor, pressure sensor, and
 * hopper wall pistons.
 * 
 * Instead of interacting with subsystems like the feeder and intake directly, the {@link Robot} class interacts with
 * the superstructure, which passes on the commands to the correct subsystem.
 * 
 * The superstructure also coordinates actions between different subsystems like the feeder and shooter.
 * 
 * @see Intake
 * @see Shooter
 * @see Climber
 * @see ColorWheel
 * @see LED
 * @see Subsystem
 */
public class Superstructure extends Subsystem {

    public enum WantedWristPosition {
    	CARGOPICKUP,   
        STRAIGHTAHEAD, // moving
        SHOOTHIGH,
        STARTINGPOSITION
    }
    
	static Superstructure mInstance = null;

    public static Superstructure getInstance() {
        if (mInstance == null) {
            mInstance = new Superstructure();
        }
        return mInstance;
    }

    private final Intake mIntake = Intake.getInstance();
    private final Shooter mShooter = Shooter.getInstance();
    private final ColorWheel mColorWheel = ColorWheel.getInstance();
    
    private final DoubleSolenoid mTopRoller = Constants.makeDoubleSolenoidForIds(1, Constants.kTopRoller1, Constants.kTopRoller2);
    private final DoubleSolenoid mBeakSwinger = Constants.makeDoubleSolenoidForIds(0, Constants.kBeakSwinger1, Constants.kBeakSwinger2);

//DEBUG ONLY - NOT FOR COMPETITION - PUT THIS BACK IN!!!!!!!!!
    private final DoubleSolenoid mBeakLips = Constants.makeDoubleSolenoidForIds(1, Constants.kBeakOpener1, Constants.kBeakOpener2);
   // private final DoubleSolenoid mBeakLips = Constants.makeDoubleSolenoidForIds(0, 3, 2);
    
    
    private final DoubleSolenoid mMustache = Constants.makeDoubleSolenoidForIds(0, Constants.kMustache1, Constants.kMustache2);
    //private final DoubleSolenoid mRotateWristShort = Constants.makeDoubleSolenoidForIds(1, Constants.kRotateWristShort1, Constants.kRotateWristShort2);
    //private final DoubleSolenoid mRotateWristLong = Constants.makeDoubleSolenoidForIds(1, Constants.kRotateWristLong1, Constants.kRotateWristLong2); 

    //private final Compressor mCompressor = new Compressor(0);
    //private final RevRoboticsAirPressureSensor mAirPressureSensor = new RevRoboticsAirPressureSensor(3);
    //private final Climber mClimber = Climber.getInstance();
    
   // private final Wrist mWrist = Wrist.getInstance();

    // Superstructure doesn't own the drive, but needs to access it
    //private final Drive mDrive = Drive.getInstance();

    // Intenal state of the system
    public enum SystemState {
        IDLE,
        WAITING_FOR_LOW_POSITION,
        WAITING_FOR_HIGH_POSITION,
        WAITING_FOR_POWERCELL_INTAKE,
        POWERCELL_INTAKING,
        POWERCELL_EJECTING,
        SHOOTING,
        CLIMB_ENGAGING,
        CLIMB_EXTENDING,
        CLIMB_RETRACTING,
        COLORWHEEL_ROTATING,
        COLORWHEEL_MATCHING,
        SPITTING,
        WAITING_FOR_ROTATE,
        SPITTING_OUT_TOP, 
        RETURNINGFROMINTAKE,
        RETURNING_HOME,
        //ELEVATOR_TRACKING,
        //EJECTING_HATCH,
        //EJECTING_CARGO,
        //CAPTURING_HATCH,
        //HATCH_CAPTURED,
        STARTINGCONFIGURATION
    };

    // Desired function from user
    public enum WantedState {
        IDLE,
        POWERCELL_INTAKE,
        POWERCELL_EJECT,
        SHOOT,
        CLIMB_ENGAGE, 
        CLIMB_EXTEND, 
        CLIMB_RETRACT, 
        COLORWHEEL_ROTATE,
        COLORWHEEL_MATCH,
        //INTAKING,
        AUTOINTAKING,
        SPITTING,
        //OVERTHETOP,
        //ELEVATOR_TRACKING,
        //HATCH_CAPTURED,
        //EJECTING_CARGO,
        //EJECTING_HATCH,
        STARTINGCONFIGURATION
    }

    private SystemState mSystemState = SystemState.IDLE;
    private WantedState mWantedState = WantedState.IDLE;

    private double mCurrentStateStartTime;
    private boolean mStateChanged;
    //private double mWantedElevatorPosition = Constants.kElevatorHomeEncoderValue;
    //private double mIntakeOutput = 0;
    private Loop mLoop = new Loop() {

        // Every time we transition states, we update the current state start
        // time and the state changed boolean (for one cycle)
        private double mWantStateChangeStartTime;

        @Override
        public void onStart(double timestamp) {
            synchronized (Superstructure.this) {
                mWantedState = WantedState.IDLE;
                mCurrentStateStartTime = timestamp;
                mWantStateChangeStartTime = timestamp;
                mSystemState = SystemState.IDLE;
                mStateChanged = true;
            }
        }

        @Override
        public void onLoop(double timestamp) {
            synchronized (Superstructure.this) {
                SystemState newState = mSystemState;
                switch (mSystemState) {
                case IDLE:
                    newState = handleIdle(mStateChanged);
                    break;
                case POWERCELL_INTAKING:
                    newState = handleIntaking();
                    break;
                case POWERCELL_EJECTING:
                    newState = handleEjecting();
                    break;
                case SHOOTING:
                    newState = handleShooting();
                    break;
                case COLORWHEEL_ROTATING:
                    newState = handleColorWheel(ColorWheel.WantedState.ROTATE); //handleColorWheelRotating();
                    break;
                case COLORWHEEL_MATCHING:
                    newState = handleColorWheel(ColorWheel.WantedState.MATCH); //handleColorWheelMatching();
                    break;
                case CLIMB_ENGAGING:
                    newState = handleClimb(Shooter.WantedState.CLIMB_ENGAGE); // handleClimbEngaging();
                    break;
                case CLIMB_EXTENDING:
                    newState = handleClimb(Shooter.WantedState.CLIMB_EXTEND); // handleClimbExtending();
                    break;
                case CLIMB_RETRACTING:
                    newState = handleClimb(Shooter.WantedState.CLIMB_RETRACT); // handleClimbRetracting();
                    break;
                /*    
                case WAITING_FOR_LOW_POSITION:
                    newState = handleWaitingForLowPosition();
                    break;
                case WAITING_FOR_HIGH_POSITION:
                    newState = handleWaitingForHightPosition();
                    break;
                case WAITING_FOR_POWERCELL_INTAKE:
                    newState = waitingForPowerCubeIntake();
                    break;
                case STARTINGCONFIGURATION:
                    newState = handleStartingConfiguration();
                    break;
                
                case WAITING_FOR_ROTATE:
                    newState = handleWaitingForRotate(timestamp);
                    break;
                case SPITTING_OUT_TOP:
                    newState = handleSpittingOutTop();
                    break;
                case ELEVATOR_TRACKING:
                    newState = handleElevatorTracking(timestamp);
                    break;
                case RETURNING_HOME:
                    newState = handleReturningHome();
                    break;
                case HATCH_CAPTURED:
                    newState = handleHatchCapture();
                    break;
                case EJECTING_HATCH:
                    newState = handleEjectingHatch();
                    break;
                case EJECTING_CARGO:
                    newState = handleEjectingCargo();
                    break;
                */
                default:
                    newState = SystemState.IDLE;
                }

                if (newState != mSystemState) {
                    System.out.println("Superstructure state " + mSystemState + " to " + newState + " Timestamp: "
                            + Timer.getFPGATimestamp());
                    mSystemState = newState;
                    mCurrentStateStartTime = timestamp;
                    mStateChanged = true;
                } else {
                    mStateChanged = false;
                }
            }
        }
/*
        private SystemState handleEjectingCargo() {
            mBeakSwinger.set(DoubleSolenoid.Value.kReverse);
            mBeakLips.set(DoubleSolenoid.Value.kReverse);
            mTopRoller.set(DoubleSolenoid.Value.kReverse);
            mMustache.set(DoubleSolenoid.Value.kReverse);
            mIntake.setWantedState(Intake.WantedState.EJECT);
        	
            switch (mWantedState) {
            case CLIMB_EXTEND:
                return SystemState.CLIMB_EXTENDING;
            case CLIMB_RETRACT:
                return SystemState.CLIMB_RETRACTING;
            case AUTOINTAKING:
                return SystemState.WAITING_FOR_LOW_POSITION;
            case INTAKING:
                return SystemState.WAITING_FOR_POWERCELL_INTAKE;
            case SPITTING:
                return SystemState.SPITTING;
            case SHOOT:
                return SystemState.SHOOTING;
            case STARTINGCONFIGURATION:
                return SystemState.STARTINGCONFIGURATION;
            case OVERTHETOP:
                return SystemState.WAITING_FOR_HIGH_POSITION;
            case ELEVATOR_TRACKING:
                return SystemState.ELEVATOR_TRACKING;
            case HATCH_CAPTURED:
                return SystemState.HATCH_CAPTURED;
            case EJECTING_HATCH:
                return SystemState.EJECTING_HATCH;
            case POWERCELL_INTAKE:
                return SystemState.POWERCELL_INTAKING;
            case POWERCELL_EJECT:
                return SystemState.POWERCELL_EJECTING;
            default:
                return SystemState.IDLE;
            }
        }
*/
        private SystemState handleStartingConfiguration(){
            //mBeakSwinger.set(DoubleSolenoid.Value.kForward);
            //mBeakLips.set(DoubleSolenoid.Value.kReverse);
            //mTopRoller.set(DoubleSolenoid.Value.kForward);
            //mMustache.set(DoubleSolenoid.Value.kReverse);
            //mClimber.setWantedState(Climber.WantedState.IDLE);
            // mWrist.setWantedPosition(WristPositions.STARTINGPOSITION);
            //seWristtWantedPosition(WantedWristPosition.STARTINGPOSITION);

            switch (mWantedState) {
            case CLIMB_EXTEND:
                return SystemState.CLIMB_EXTENDING;
            case CLIMB_RETRACT:
                return SystemState.CLIMB_RETRACTING;
            case AUTOINTAKING:
                return SystemState.WAITING_FOR_LOW_POSITION;
            //case INTAKING:
            //    return SystemState.WAITING_FOR_POWERCELL_INTAKE;
            case SPITTING:
                return SystemState.SPITTING;
            case SHOOT:
                return SystemState.SHOOTING;
            case STARTINGCONFIGURATION:
                return SystemState.STARTINGCONFIGURATION;
            case POWERCELL_EJECT:
                return SystemState.POWERCELL_EJECTING;
            /*
            case OVERTHETOP:
                return SystemState.WAITING_FOR_HIGH_POSITION;
            case ELEVATOR_TRACKING:
                return SystemState.ELEVATOR_TRACKING;
            case HATCH_CAPTURED:
                return SystemState.HATCH_CAPTURED;
            case EJECTING_HATCH:
                return SystemState.EJECTING_HATCH;
            */
            case POWERCELL_INTAKE:
                return SystemState.POWERCELL_INTAKING;
            default:
                return SystemState.IDLE;
            }
        }

        private SystemState handleIntaking() {
            //mBeakSwinger.set(DoubleSolenoid.Value.kReverse);
            //mBeakLips.set(DoubleSolenoid.Value.kReverse);
            //mTopRoller.set(DoubleSolenoid.Value.kForward);
            //mMustache.set(DoubleSolenoid.Value.kReverse);
            mIntake.setWantedState(Intake.WantedState.INTAKE);
            //seWristtWantedPosition(WantedWristPosition.CARGOPICKUP);
            //setWantedElevatorPosition(ELEVATOR_POSITION.ELEVATOR_CARGO_PICKUP);
            //mElevator.setWantedPosition(Constants.kElevatorBallPickup_EncoderValue);

            switch (mWantedState) {
                case POWERCELL_INTAKE:
                    return SystemState.POWERCELL_INTAKING;
                case POWERCELL_EJECT:
                    return SystemState.POWERCELL_EJECTING;
                case SHOOT:
                    return SystemState.SHOOTING;
                case COLORWHEEL_ROTATE:
                    return SystemState.COLORWHEEL_ROTATING;
                case COLORWHEEL_MATCH:
                    return SystemState.COLORWHEEL_MATCHING;
                case CLIMB_ENGAGE:
                    return SystemState.CLIMB_EXTENDING;
                case CLIMB_EXTEND:
                    return SystemState.CLIMB_EXTENDING;
                case CLIMB_RETRACT:
                    return SystemState.CLIMB_RETRACTING;
                case AUTOINTAKING:
                    return SystemState.WAITING_FOR_LOW_POSITION;
                case STARTINGCONFIGURATION:
                    return SystemState.STARTINGCONFIGURATION;
                default:
                    return SystemState.IDLE;
            /*    
            case INTAKING:
                return SystemState.WAITING_FOR_POWERCELL_INTAKE;
            case SPITTING:
                return SystemState.SPITTING;
            case OVERTHETOP:
                return SystemState.WAITING_FOR_HIGH_POSITION;
            case ELEVATOR_TRACKING:
                mTopRoller.set(DoubleSolenoid.Value.kReverse);
                seWristtWantedPosition(WantedWristPosition.STRAIGHTAHEAD);
                return SystemState.ELEVATOR_TRACKING;
            case HATCH_CAPTURED:
                return SystemState.HATCH_CAPTURED;
            case EJECTING_HATCH:
                return SystemState.EJECTING_HATCH;
            */
            }
        }

        private SystemState handleColorWheel(ColorWheel.WantedState colorWheelState) {
            //mColorWheelPiston.set(DoubleSolenoid.Value.kReverse);
            mColorWheel.setWantedState(colorWheelState);

            switch (mWantedState) {
                case POWERCELL_INTAKE:
                    return SystemState.POWERCELL_INTAKING;
                case POWERCELL_EJECT:
                    return SystemState.POWERCELL_EJECTING;
                case SHOOT:
                    return SystemState.SHOOTING;
                case COLORWHEEL_ROTATE:
                    return SystemState.COLORWHEEL_ROTATING;
                case COLORWHEEL_MATCH:
                    return SystemState.COLORWHEEL_MATCHING;
                case CLIMB_ENGAGE:
                    return SystemState.CLIMB_EXTENDING;
                case CLIMB_EXTEND:
                    return SystemState.CLIMB_EXTENDING;
                case CLIMB_RETRACT:
                    return SystemState.CLIMB_RETRACTING;
                case AUTOINTAKING:
                    return SystemState.WAITING_FOR_LOW_POSITION;
                case STARTINGCONFIGURATION:
                    return SystemState.STARTINGCONFIGURATION;
                default:
                    return SystemState.IDLE;
            }
        }

		private SystemState handleShooting() {
            mShooter.setWantedState(Shooter.WantedState.SHOOT);
            mIntake.setWantedState(Intake.WantedState.SHOOT);
        	
            switch (mWantedState) {
                case POWERCELL_INTAKE:
                    return SystemState.POWERCELL_INTAKING;
                case POWERCELL_EJECT:
                    return SystemState.POWERCELL_EJECTING;
                case SHOOT:
                    return SystemState.SHOOTING;
                case COLORWHEEL_ROTATE:
                    return SystemState.COLORWHEEL_ROTATING;
                case COLORWHEEL_MATCH:
                    return SystemState.COLORWHEEL_MATCHING;
                case CLIMB_ENGAGE:
                    return SystemState.CLIMB_EXTENDING;
                case CLIMB_EXTEND:
                    return SystemState.CLIMB_EXTENDING;
                case CLIMB_RETRACT:
                    return SystemState.CLIMB_RETRACTING;
                case AUTOINTAKING:
                    return SystemState.WAITING_FOR_LOW_POSITION;
                case STARTINGCONFIGURATION:
                    return SystemState.STARTINGCONFIGURATION;
                default:
                    return SystemState.IDLE;
            /*        
            case INTAKING:
                return SystemState.WAITING_FOR_POWERCELL_INTAKE;
            case SPITTING:
                return SystemState.SPITTING;
            case OVERTHETOP:
                return SystemState.SPITTING_OUT_TOP;
            case HATCH_CAPTURED:
                return SystemState.HATCH_CAPTURED;
            case EJECTING_HATCH:
                return SystemState.EJECTING_HATCH;

            */
            }
        }

		private SystemState handleEjecting() {
            mIntake.setWantedState(Intake.WantedState.EJECT);
        	
            switch (mWantedState) {
                case POWERCELL_INTAKE:
                    return SystemState.POWERCELL_INTAKING;
                case POWERCELL_EJECT:
                    return SystemState.POWERCELL_EJECTING;
                case SHOOT:
                    return SystemState.SHOOTING;
                case COLORWHEEL_ROTATE:
                    return SystemState.COLORWHEEL_ROTATING;
                case COLORWHEEL_MATCH:
                    return SystemState.COLORWHEEL_MATCHING;
                case CLIMB_ENGAGE:
                    return SystemState.CLIMB_EXTENDING;
                case CLIMB_EXTEND:
                    return SystemState.CLIMB_EXTENDING;
                case CLIMB_RETRACT:
                    return SystemState.CLIMB_RETRACTING;
                case AUTOINTAKING:
                    return SystemState.WAITING_FOR_LOW_POSITION;
                case STARTINGCONFIGURATION:
                    return SystemState.STARTINGCONFIGURATION;
                default:
                    return SystemState.IDLE;
            }
        }

        /*
		private SystemState handleWaitingForHightPosition() {
			return SystemState.IDLE;
		}

		private SystemState handleWaitingForLowPosition() {
        	mElevator.setWantedState(Elevator.WantedState.ELEVATORTRACKING);
        	mIntake.setWantedState(Intake.WantedState.INTAKE);
        	
            switch (mWantedState) {
            case CLIMB_EXTEND:
                return SystemState.CLIMB_EXTENDING;
            case CLIMB_RETRACT:
                return SystemState.CLIMB_RETRACTING;
            case AUTOINTAKING:{
            	if (mElevator.atBottom())
            		return SystemState.WAITING_FOR_POWERCELL_INTAKE; 
            	else  
                return SystemState.WAITING_FOR_LOW_POSITION;
            	}
            	
            case INTAKING:
                return SystemState.WAITING_FOR_POWERCELL_INTAKE;
            case SPITTING:
                return SystemState.SPITTING;
            case SHOOT:
                return SystemState.SHOOTING;
            case STARTINGCONFIGURATION:
                return SystemState.STARTINGCONFIGURATION;
            case POWERCELL_EJECT:
                return SystemState.POWERCELL_EJECTING;
            case OVERTHETOP:
                return SystemState.SPITTING_OUT_TOP;
            case ELEVATOR_TRACKING:
                return SystemState.ELEVATOR_TRACKING;
            case HATCH_CAPTURED:
                return SystemState.HATCH_CAPTURED;
            case EJECTING_HATCH:
                return SystemState.EJECTING_HATCH;
            case POWERCELL_INTAKE:
                return SystemState.POWERCELL_INTAKING;
            default:
                return SystemState.IDLE;
            }
        }
        */
        private SystemState handleClimb(Shooter.WantedState shooterState) { //handleClimbEngaging() {
            mShooter.setWantedState(shooterState); //Shooter.WantedState.CLIMB_ENGAGE);

            switch (mWantedState) {
                case POWERCELL_INTAKE:
                    return SystemState.POWERCELL_INTAKING;
                case POWERCELL_EJECT:
                    return SystemState.POWERCELL_EJECTING;
                case SHOOT:
                    return SystemState.SHOOTING;
                case COLORWHEEL_ROTATE:
                    return SystemState.COLORWHEEL_ROTATING;
                case COLORWHEEL_MATCH:
                    return SystemState.COLORWHEEL_MATCHING;
                case CLIMB_ENGAGE:
                    return SystemState.CLIMB_EXTENDING;
                case CLIMB_EXTEND:
                    return SystemState.CLIMB_EXTENDING;
                case CLIMB_RETRACT:
                    return SystemState.CLIMB_RETRACTING;
                case AUTOINTAKING:
                    return SystemState.WAITING_FOR_LOW_POSITION;
                case STARTINGCONFIGURATION:
                    return SystemState.STARTINGCONFIGURATION;
                default:
                    return SystemState.IDLE;
            }
        }

		@Override
        public void onStop(double timestamp) {
            stop();
        }
    };

    private SystemState handleIdle(boolean stateChanged) {
        if (stateChanged) {
            stop();
            //mMustache.set(DoubleSolenoid.Value.kReverse);
            //mElevator.setWantedState(Elevator.WantedState.IDLE);
            mIntake.setWantedState(Intake.WantedState.IDLE);
            mShooter.setWantedState(Shooter.WantedState.IDLE);
        }
        
        switch (mWantedState) {
            case POWERCELL_INTAKE:
                return SystemState.POWERCELL_INTAKING;
            case POWERCELL_EJECT:
                return SystemState.POWERCELL_EJECTING;
            case SHOOT:
                return SystemState.SHOOTING;
            case COLORWHEEL_ROTATE:
                return SystemState.COLORWHEEL_ROTATING;
            case COLORWHEEL_MATCH:
                return SystemState.COLORWHEEL_MATCHING;
            case CLIMB_ENGAGE:
               return SystemState.CLIMB_EXTENDING;
            case CLIMB_EXTEND:
                return SystemState.CLIMB_EXTENDING;
            case CLIMB_RETRACT:
                return SystemState.CLIMB_RETRACTING;
            case AUTOINTAKING:
                return SystemState.WAITING_FOR_LOW_POSITION;
        /*
        case STARTINGCONFIGURATION:
            return SystemState.STARTINGCONFIGURATION;
        case OVERTHETOP:
            return SystemState.WAITING_FOR_HIGH_POSITION;
        case ELEVATOR_TRACKING:
            return SystemState.ELEVATOR_TRACKING;
        case HATCH_CAPTURED:
            return SystemState.HATCH_CAPTURED;
        case EJECTING_HATCH:
            return SystemState.EJECTING_HATCH;
        case INTAKING:
            return SystemState.WAITING_FOR_POWERCELL_INTAKE;      
        case SPITTING:
            return SystemState.SPITTING;
        */
            default:
                return SystemState.IDLE;
        }
    }

    public synchronized void setWantedState(WantedState wantedState) {
        mWantedState = wantedState;
    }

    @Override
    public void outputToSmartDashboard() {
        //SmartDashboard.putNumber("Air Pressure psi", mAirPressureSensor.getAirPressurePsi());
        SmartDashboard.putString("Sys State", mSystemState.name());
        //SmartDashboard.putNumber("IntakeOutput", mIntakeOutput);
        mIntake.outputToSmartDashboard();
        mShooter.outputToSmartDashboard();
    }

    @Override
    public void stop() {
    }

    @Override
    public void zeroSensors() {
    }

    @Override
    public void registerEnabledLoops(Looper enabledLooper) {
        enabledLooper.register(mLoop);
    }
    /*
    public void closeBeak() {
        mBeakLips.set(DoubleSolenoid.Value.kReverse);
    }
    public void openBeak() {
        mBeakLips.set(DoubleSolenoid.Value.kForward);
    }

    public void prepareToPickupHatch() {
        mBeakSwinger.set(DoubleSolenoid.Value.kForward);
        mBeakLips.set(DoubleSolenoid.Value.kReverse);
        mTopRoller.set(DoubleSolenoid.Value.kReverse);
        mMustache.set(DoubleSolenoid.Value.kReverse);
    }
   

    public void ejectHatch() {
        mBeakSwinger.set(DoubleSolenoid.Value.kForward);
        mBeakLips.set(DoubleSolenoid.Value.kReverse);
        mTopRoller.set(DoubleSolenoid.Value.kReverse);
        mMustache.set(DoubleSolenoid.Value.kForward);
    }

    public void uproller() {
        mTopRoller.set(DoubleSolenoid.Value.kReverse);
        seWristtWantedPosition(WantedWristPosition.STRAIGHTAHEAD);
    }
    */
    public void setOverrideCompressor(boolean force_off) {
    }

    public void reloadConstants() {
    }

}
