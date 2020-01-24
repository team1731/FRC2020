package frc.robot;

import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;
import java.util.Optional;

import org.usfirst.frc.team1731.lib.util.CheesyDriveHelper;
import org.usfirst.frc.team1731.lib.util.CrashTracker;
import org.usfirst.frc.team1731.lib.util.DelayedBoolean;
import org.usfirst.frc.team1731.lib.util.DriveSignal;
import org.usfirst.frc.team1731.lib.util.InterpolatingDouble;
import org.usfirst.frc.team1731.lib.util.InterpolatingTreeMap;
import org.usfirst.frc.team1731.lib.util.LatchedBoolean;
import org.usfirst.frc.team1731.lib.util.math.RigidTransform2d;
import org.usfirst.frc.team1731.robot.Constants;
import org.usfirst.frc.team1731.robot.ControlBoardInterface;
import org.usfirst.frc.team1731.robot.GamepadControlBoard;
import org.usfirst.frc.team1731.robot.ShooterAimingParameters;
import org.usfirst.frc.team1731.robot.SubsystemManager;
import org.usfirst.frc.team1731.robot.Constants.GRABBER_POSITION;
import org.usfirst.frc.team1731.robot.auto.AutoModeBase;
import org.usfirst.frc.team1731.robot.auto.AutoModeExecuter;
import org.usfirst.frc.team1731.robot.auto.modes.*;
import org.usfirst.frc.team1731.robot.auto.modes._new.*;
import org.usfirst.frc.team1731.robot.auto.modes.spacey.*;
import org.usfirst.frc.team1731.robot.loops.JevoisVisionProcessor;
import org.usfirst.frc.team1731.robot.loops.Looper;
import org.usfirst.frc.team1731.robot.loops.RobotStateEstimator;
import org.usfirst.frc.team1731.robot.loops.VisionProcessor;
import org.usfirst.frc.team1731.robot.paths.DriveForward;
import org.usfirst.frc.team1731.robot.paths.profiles.PathAdapter;
import org.usfirst.frc.team1731.robot.subsystems.ConnectionMonitor;
import org.usfirst.frc.team1731.robot.subsystems.Drive;
import org.usfirst.frc.team1731.robot.subsystems.Elevator;
import org.usfirst.frc.team1731.robot.subsystems.Climber;
import org.usfirst.frc.team1731.robot.subsystems.Intake;
import org.usfirst.frc.team1731.robot.subsystems.LED;
import org.usfirst.frc.team1731.robot.subsystems.Superstructure;
import org.usfirst.frc.team1731.robot.vision.JevoisVisionServer;
import org.usfirst.frc.team1731.robot.vision.VisionServer;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoSink;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobot;
//import edu.wpi.first.wpilibj.RobotState;
import org.usfirst.frc.team1731.robot.RobotState;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The main robot class, which instantiates all robot parts and helper classes and initializes all loops. Some classes
 * are already instantiated upon robot startup; for those classes, the robot gets the instance as opposed to creating a
 * new object
 * 
 * After initializing all robot parts, the code sets up the autonomous and teleoperated cycles and also code that runs
 * periodically inside both routines.
 * 
 * This is the nexus/converging point of the robot code and the best place to start exploring.
 * 
 * The VM is configured to automatically run this class, and to call the functions corresponding to each mode, as
 * described in the IterativeRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the manifest file in the resource directory.
 */

//original code was 2018 -- I brought in 2019 chunks as indicated...
public class RobotNew extends TimedRobot {
	//2019
    private DigitalOutput leftRightCameraControl;
    private DigitalOutput greenLEDRingLight;
	
	public static enum AutoScheme { 
		OLD_SCHEME, // Haymarket, Alexandria
		NEW_SCHEME  // Maryland, Detroit
	}
	public static AutoScheme CHOSEN_AUTO_SCHEME = AutoScheme.NEW_SCHEME; // or, AutoScheme.OLD_SCHEME
		
	private static final String AUTO_CODE = "AutoCode";
	private static final String AUTO_CODES = "AutoCodes";
    private static Map<Integer, AutoModeBase> AUTO_MODES; // 35 modes defined in Mark's "BIBLE"
    private static Map<String, AutoModeBase> AUTO_MODES_2019; // 35 modes defined in Mark's "BIBLE"
    private static Map<String, String[]> ALLOWABLE_AUTO_MODES; //  as defined in Mark's "BIBLE"
    
	static {
		initAutoModes2018();
        initAllowableAutoModes();
	}
    
    
	// example from 2019:
	private static AutoModeBase newLeftRocketRearToFeedStationMode  = new LeftRocketRearToFeedStationMode();

	
	
	//[]TODO: modify this to retrieve color -- example is on slack
	public static String getGameDataFromField() {     // "LLR" for example
        String gameData = DriverStation.getInstance().getGameSpecificMessage().trim();
        int retries = 100;
          	
        while (gameData.length() < 2 && retries > 0) {
            retries--;
            try {
                Thread.sleep(5);
            } catch (InterruptedException ie) {
                // Just ignore the interrupted exception
            }
            gameData = DriverStation.getInstance().getGameSpecificMessage().trim();
        }
        if(gameData.length() < 2) {
        	gameData = "LR";
        }
        return gameData;
	}
	

	//2018
	// Get subsystem instances
    //private Drive mDrive = Drive.getInstance();
	//private Climber mClimber = Climber.getInstance();
    //private Superstructure mSuperstructure = Superstructure.getInstance();
    //private LED mLED = LED.getInstance();
    //private RobotState mRobotState = RobotState.getInstance();
    //private AutoModeExecuter mAutoModeExecuter = null;
    private AutoModeBase autoModeToExecute;
    private SendableChooser autoChooser;
    
	//2019
	// Get subsystem instances
	//[]TODO: modify this section to get a SWERVE instance
    private Drive mDrive = Drive.getInstance();
    private Climber mClimber = Climber.getInstance();
    private Superstructure mSuperstructure = Superstructure.getInstance();
    private org.usfirst.frc.team1731.robot.RobotState mRobotState = org.usfirst.frc.team1731.robot.RobotState
            .getInstance();
    private AutoModeExecuter mAutoModeExecuter = null;
    private RobotStateEstimator mRobotStateEstimator = RobotStateEstimator.getInstance();
    private JevoisVisionProcessor mVisionCamProcessor = JevoisVisionProcessor.getInstance();

	//2019
	private AutoModeBase[] autoModesToExecute;
    private boolean joystickAxesAreReversed;
    private boolean driveSpeedIsToggled;
    private boolean camerasAreReversed;
    private boolean tractorIndicator = Boolean.FALSE;  
    private boolean mLoopersAreRunning = false;       
    private UsbCamera cameraFront;
    private DigitalOutput arduinoLed0;
    private DigitalOutput arduinoLed1;
    private DigitalOutput arduinoLed2;
    private Boolean invertCameraPrevious = Boolean.FALSE;
    private Boolean mTractorBeamPickupSelected = false;
    private Boolean mTractorBeamHatchSelected = false;
    private VideoSink videoSink;
    private String autoCode = Constants.kDefaultAutoMode;
    private JevoisVisionServer mVisionServer = JevoisVisionServer.getInstance();


	//2018
    private SendableChooser startingPosition;
    private SendableChooser areTeammatesCool;
    private enum startingPositions {
    	LEFT,
 //   	MIDDLELEFT,
    	MIDDLERIGHT,
    	RIGHT
    };

	//2018
    private final SubsystemManager mSubsystemManager2018 = new SubsystemManager(
                            Arrays.asList(
								Drive.getInstance(),
								Superstructure.getInstance(),
                                Elevator.getInstance(),
								Intake.getInstance(),
								Climber.getInstance(),
                                ConnectionMonitor.getInstance(),
								LED.getInstance() ));

	//2019
    private final SubsystemManager mSubsystemManager = new SubsystemManager(
                           Arrays.asList(
                                         Drive.getInstance(),
                                         Superstructure.getInstance(),
                                         Elevator.getInstance(),
                                         Intake.getInstance(),
                                         Climber.getInstance()
                                         ));
	
	
	
    // Initialize other helper objects
    private CheesyDriveHelper mCheesyDriveHelper = new CheesyDriveHelper();
    private ControlBoardInterface mControlBoard = GamepadControlBoard.getInstance();

    private Looper mEnabledLooper = new Looper();
    private InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> mTuningFlywheelMap = new InterpolatingTreeMap<>();

    private AnalogInput mCheckLightButton = new AnalogInput(Constants.kLEDOnId);
    private Double mTractorBeamGain = 1.0;

	//2019
	private DigitalInput tapeSensor;
    private double disabledTimestampSave;
    private enum AUTO_MODE_SEL {
        ROCKET_REAR_TO_FEED_STATION(0),
        FEED_STATION_TO_ROCKET_REAR(1),
        ROCKET_FRONT_TO_FEED_STATION(2),
        FEED_STATION_TO_ROCKET_FRONT(3);

        private final int arrayPosition;

        private AUTO_MODE_SEL(int arrayPosition){
            this.arrayPosition = arrayPosition;
        }

        private int getArrayPosition(){
            return arrayPosition;
        }
    }
	
	
	
    public RobotNew() {
		//2019
		super(0.2);
		
        CrashTracker.logRobotConstruction();
    }

    public void zeroAllSensors() {
        mSubsystemManager.zeroSensors();
        mRobotState.reset(Timer.getFPGATimestamp(), new RigidTransform2d());
       // mDrive.zeroSensors(); subsystem manager does this
    }

    /**
     * This function is run when the robot is first started up and should be used for any initialization code.
     */
    @Override
    public void robotInit() {
        try {
            CrashTracker.logRobotInit();
            
            final double kVoltageThreshold = 0.15;
            if (mCheckLightButton.getAverageVoltage() < kVoltageThreshold) {
                //mLED.setLEDOn();
            } else {
                //mLED.setLEDOff();
            }

            String tractorGain = SmartDashboard.getString("TractorGain", "1.0");

            autoCode = SmartDashboard.getString("AutoCode", Constants.kDefaultAutoMode); // or R
    
            greenLEDRingLight = new DigitalOutput(0);
            greenLEDRingLight.set(false); // turn off the light until teleop

            arduinoLed0 = new DigitalOutput(Constants.kArduinoLed0);
            arduinoLed1 = new DigitalOutput(Constants.kArduinoLed1);
            arduinoLed2 = new DigitalOutput(Constants.kArduinoLed2);


            mSubsystemManager.registerEnabledLoops(mEnabledLooper);
            mEnabledLooper.register(mRobotStateEstimator);
            mEnabledLooper.register(mVisionCamProcessor); 
            
            //http://roborio-1731-frc.local:1181/?action=stream
            //   /CameraPublisher/<camera name>/streams=["mjpeg:http://roborio-1731-frc.local:1181/?action=stream", "mjpeg:http://10.17.31.2:1181/?action=stream"]
            try {
                cameraFront = CameraServer.getInstance().startAutomaticCapture(0);
                //cameraBack = CameraServer.getInstance().startAutomaticCapture(1);
                videoSink = CameraServer.getInstance().getServer();
                //selectedCamera = cameraFront;
            }
            catch (Throwable t) {
                System.out.println("Exception while connecting driver camera: " + t.toString());
            }
            
            
            switch(CHOSEN_AUTO_SCHEME) {
            
            case OLD_SCHEME: // Haymarket, Alexandria
                autoChooser = new SendableChooser();
                autoChooser.addDefault("Score Cubes", "ScoreCubes");
                autoChooser.addObject("Drive and do nothing", "DriveOnly");
                autoChooser.addObject("Do Nothing", new StandStillMode());
                autoChooser.addObject("Test", new TestAuto());
                autoChooser.addObject("3 on Right Scale", new RightPutCubeOnRightScale());
                autoChooser.addObject("3 on Left Scale", new RightPut3CubesOnLeftScale());
                SmartDashboard.putData("Autonomous Mode", autoChooser);
               
                startingPosition = new SendableChooser();
                startingPosition.addDefault("Left Position", startingPositions.LEFT);
//                startingPosition.addObject("Middle-Left Position", startingPositions.MIDDLELEFT);
                startingPosition.addObject("Middle-Right Position", startingPositions.MIDDLERIGHT);
                startingPosition.addObject("Right Position", startingPositions.RIGHT);
                SmartDashboard.putData("Starting Position", startingPosition);
                
                areTeammatesCool = new SendableChooser();
                areTeammatesCool.addDefault("Be cautious (A)", false);
                areTeammatesCool.addObject("It's fine (B)", true);
                SmartDashboard.putData("How should I react?", areTeammatesCool);
                
            	break;
            	
            case NEW_SCHEME: // Maryland, Detroit             //LL LR RL RR
            	SmartDashboard.putString(AUTO_CODES, "3  8 12 15");
            	break;
            }
            
			//2019
			SmartDashboard.putString(AUTO_CODE, Constants.kDefaultAutoMode);
            SmartDashboard.putString("TractorGain", "1.2");   


			//20018
            // Pre calculate the paths we use for auto.
            PathAdapter.calculatePaths();

        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
		
		//2018
        zeroAllSensors();
		
		//2019
		//zeroAllSensors();
    }

    /**
     * Initializes the robot for the beginning of autonomous mode (set drivebase, intake and superstructure to correct
     * states). Then gets the correct auto mode from the AutoModeSelector
     * 
     * @see AutoModeSelector.java
     */
    @Override
    public void autonomousInit() {
        try {
            CrashTracker.logAutoInit();

            System.out.println("Auto start timestamp: " + Timer.getFPGATimestamp());

            if (mAutoModeExecuter != null) {
                mAutoModeExecuter.stop();
            }

            zeroAllSensors();
            mSuperstructure.setWantedState(Superstructure.WantedState.IDLE);
            mSuperstructure.setOverrideCompressor(true);

            mAutoModeExecuter = null;
			
			//2019
            mDrive.zeroSensors();
			
            // Shift to high
            mDrive.setHighGear(true);
            mDrive.setBrakeMode(true);

            mEnabledLooper.start();
			mLoopersAreRunning = true;
            mSuperstructure.reloadConstants();
            
			//2019
			mSuperstructure.openBeak();
            mSuperstructure.uproller();

            switch(CHOSEN_AUTO_SCHEME) {
            
            case OLD_SCHEME: // Haymarket, Alexandria

	            if (autoChooser.getSelected().equals("ScoreCubes")) {
	            	autoModeToExecute = AutoDetectAllianceSwitchThenPlaceMode.pickAutoMode(
	            			(AutoDetectAllianceSwitchThenPlaceMode.startingPositions.valueOf(startingPosition.getSelected().toString())),
	            			(boolean) areTeammatesCool.getSelected());
	            
	            } else if(autoChooser.getSelected().equals("DriveOnly")) {
	            	autoModeToExecute = AutoDetectAllianceSwitchThenPlaceMode.intenseTrust(
	            			AutoDetectAllianceSwitchThenPlaceMode.startingPositions.valueOf(startingPosition.getSelected().toString()));
	            } else 
	            	autoModeToExecute = (AutoModeBase) autoChooser.getSelected();
	            
	            break;
	            
            case NEW_SCHEME: // Maryland, Detroit
            	
            	String gameData = RobotNew.getGameDataFromField(); //RRL for example
            	String autoCodes = SmartDashboard.getString("AutoCodes", "3  8 12 15");// JUSTIN's numbers

                autoModeToExecute = determineAutoModeToExecute2018(gameData, autoCodes);
            	break;
            }
            
            mAutoModeExecuter = new AutoModeExecuter();
            mAutoModeExecuter.setAutoMode(autoModeToExecute);
            mAutoModeExecuter.start();
            
            //WPILIB WAY TO GET AUTONOMOUS MODE...
            //
            //
            //autonomousCommand = (Command) autoChooser.getSelected();
            //autonomousCommand.start();
            

        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }
                                                  // RRL for example
    private AutoModeBase determineAutoModeToExecute2018(String gameData, String autoCodes) {
    	System.out.println("Got field configuration: " + gameData);
    	System.out.println("Got these auto modes from the dashboard: " + autoCodes);
    	                                         //LL LR RL RR
    	String[] autoCodeArray = autoCodes.split(" ");//"3  8 12 15" for example
    	String LLcode = "3";
    	String LRcode = "8";
    	String RLcode = "12";
    	String RRcode = "15";
    	if(autoCodeArray.length == 4) {
        	LLcode = autoCodeArray[0];
        	LRcode = autoCodeArray[1];
        	RLcode = autoCodeArray[2];
        	RRcode = autoCodeArray[3];
    	}
    	
        AutoModeBase selectedAutoMode = null;
        String fieldSetup = gameData.substring(0, 2);//"RR" for example
        switch(fieldSetup) {
        case "LL":
        	selectedAutoMode = lookupMode(LLcode);
        	break;
        case "LR":
        	selectedAutoMode = lookupMode(LRcode);
        	break;
        case "RL":
        	selectedAutoMode = lookupMode(RLcode);
        	break;
        case "RR":
        	selectedAutoMode = lookupMode(RRcode);
        	break;
        }
        
        selectedAutoMode = performSanityCheck(selectedAutoMode, fieldSetup);
        
        System.out.println("running auto mode: " + selectedAutoMode);
        
		return selectedAutoMode;
	}

    private AutoModeBase[] determineAutoModesToExecute2019(String autoCode) {
        //System.out.println("Got this string from the dashboard: " + autoCode);
        AutoModeBase[] autoModes = new AutoModeBase[4];
        //default is LEFT side rocket:
        //default is LEFT side rocket:
        //default is LEFT side rocket:
        autoModes[AUTO_MODE_SEL.ROCKET_REAR_TO_FEED_STATION.getArrayPosition()] = AUTO_MODES.get("1"); // ROCKET REAR TO FEED STATION
        autoModes[AUTO_MODE_SEL.FEED_STATION_TO_ROCKET_REAR.getArrayPosition()] = AUTO_MODES.get("J"); // FEED STATION TO ROCKET REAR
        autoModes[AUTO_MODE_SEL.ROCKET_FRONT_TO_FEED_STATION.getArrayPosition()] = AUTO_MODES.get("5"); // ROCKET FRONT TO FEED STATION
        autoModes[AUTO_MODE_SEL.FEED_STATION_TO_ROCKET_FRONT.getArrayPosition()] = AUTO_MODES.get("C"); // FEED STATION TO ROCKET FRONT
        if(autoCode != null && autoCode.length() > 0){
            String autoCodeChar = autoCode.toUpperCase().substring(0,1);
            if("R".equals(autoCodeChar)){
                //RIGHT side rocket:
                //RIGHT side rocket:
                //RIGHT side rocket:
                autoModes[AUTO_MODE_SEL.ROCKET_REAR_TO_FEED_STATION.getArrayPosition()] = AUTO_MODES.get("1"); // ROCKET REAR TO FEED STATION
                autoModes[AUTO_MODE_SEL.FEED_STATION_TO_ROCKET_REAR.getArrayPosition()] = AUTO_MODES.get("J"); // FEED STATION TO ROCKET REAR
                autoModes[AUTO_MODE_SEL.ROCKET_FRONT_TO_FEED_STATION.getArrayPosition()] = AUTO_MODES.get("5"); // ROCKET FRONT TO FEED STATION
                autoModes[AUTO_MODE_SEL.FEED_STATION_TO_ROCKET_FRONT.getArrayPosition()] = AUTO_MODES.get("C"); // FEED STATION TO ROCKET FRONT
            }
        }
        //System.out.println("running auto modes: " + Arrays.toString(autoModes));
		return autoModes;
	}

    private AutoModeBase performSanityCheck(AutoModeBase selectedAutoMode, String fieldSetup) {
    	String humanSelectedAutoModeName = selectedAutoMode.getClass().getSimpleName();
    	String[] allowedAutoModeNamesForThisFieldSetup = ALLOWABLE_AUTO_MODES.get(fieldSetup);
    	if(!Arrays.asList(allowedAutoModeNamesForThisFieldSetup).contains(humanSelectedAutoModeName)) {
    		return new _00_DO_NOTHING();
    	}
		return selectedAutoMode;
	}

	private static void initAutoModes2018() {
    	AUTO_MODES = new HashMap<Integer, AutoModeBase>();//THESE ARE FROM MARK'S "BIBLE"
        AUTO_MODES.put(2,  /*   Far SC X3 				 */ new _02_RightPut3LeftScale());
        AUTO_MODES.put(3,  /* 	Far SC-Far SW-Far SC 	 */ new _03_RightPut2LeftScale1LeftSwitch());
        AUTO_MODES.put(5,  /* 	Drive Forward 			 */ new _05_RightDriveForward());
        AUTO_MODES.put(7,  /* 	SC x3 					 */ new _07_RightPut3RightScale());
        AUTO_MODES.put(8,  /* 	SC-Far SW 				 */ new _08_RightPut1RightScale1LeftSwitchEnd());
        AUTO_MODES.put(9,  /* 	SC x2-Far SW 			 */ new _00_DO_NOTHING()); //new _09_RightPut2RightScale1LeftSwitch());
        AUTO_MODES.put(11, /* 	SW x2 					 */ new _11_RightPut1RightSwitchEnd1RightSwitch());
        AUTO_MODES.put(12, /* 	SW- Far SC 				 */ new _12_RightPut1RightSwitchEnd1LeftScale());
        AUTO_MODES.put(15, /* 	SC - SW - SC	 		 */ new _15_RightPut2RightScale1RightSwitch());
        AUTO_MODES.put(19, /* 	Far SC X3		 		 */ new _19_LeftPut3RightScale());
        AUTO_MODES.put(20, /* 	Far SC - Far SW - Far SC */ new _20_LeftPut2RightScale1RightSwitch());
        AUTO_MODES.put(23, /* 	SC x3					 */ new _23_LeftPut3LeftScale());
        AUTO_MODES.put(24, /* 	SC - Far SW				 */ new _24_LeftPut1LeftScale1RightSwitchEnd());
        AUTO_MODES.put(25, /* 	SC X2 - Far SW			 */ new _00_DO_NOTHING()); //new _25_LeftPut2LeftScale1RightSwitch());
        AUTO_MODES.put(27, /* 	SW x2					 */ new _27_LeftPut1LeftSwitchEnd1LeftSwitch());
        AUTO_MODES.put(28, /* 	SW - Far SC				 */ new _28_LeftPut1LeftSwitchEnd1RightScale());
        AUTO_MODES.put(31, /* 	SC - SW - SC			 */ new _31_LeftPut2LeftScale1LeftSwitch());
        AUTO_MODES.put(34, /* 	SW						 */ new _34_MiddlePut1LeftSwitch());
        AUTO_MODES.put(35, /* 	SW x2					 */ new _35_MiddlePut2LeftSwitch());
        AUTO_MODES.put(36, /* 	SW - EX					 */ new _36_MiddlePut1LeftSwitch1Exchange());
        AUTO_MODES.put(37, /* 	SW						 */ new _37_MiddlePut1RightSwitch());
        AUTO_MODES.put(38, /* 	SW x2					 */ new _38_MiddlePut2RightSwitch());
        AUTO_MODES.put(39, /* 	SW - EX					 */ new _39_MiddlePut1RightSwitch1Exchange());
        AUTO_MODES.put(40, /* 	Far SC-Far SW x2		 */ new _00_DO_NOTHING()); //new _40_RightPut1LeftScale2LeftSwitch());
        AUTO_MODES.put(41, /* 	SC End					 */ new _41_RightPut1RightScaleEnd());
        AUTO_MODES.put(42, /* 	SC End-Far SW			 */ new _42_RightPut1RightScaleEnd1LeftSwitch());
        AUTO_MODES.put(43, /* 	SC End x2				 */ new _43_RightPut2RightScaleEnd());
        AUTO_MODES.put(46, /* 	SW - SC End		 		 */ new _00_DO_NOTHING()); //new _46_RightPut1RightSwitchEnd1RightScaleEnd());
        AUTO_MODES.put(47, /* 	Drive FWD				 */ new _47_MiddleDriveForward());
        AUTO_MODES.put(48, /* 	SC End x2				 */ new _48_LeftPut2LeftScaleEnd());
        AUTO_MODES.put(49, /* 	SC End					 */ new _49_LeftPut1LeftScaleEnd());
        AUTO_MODES.put(50, /* 	SW - SC End				 */ new _00_DO_NOTHING()); //new _50_LeftPut1LeftSwitchEnd1LeftScaleEnd());
        AUTO_MODES.put(52, /* 	SC End - Far SW			 */ new _52_LeftPut1LeftScaleEnd1RightSwitch());
        AUTO_MODES.put(54, /* 	Far SC - Far SW X2		 */ new _00_DO_NOTHING()); //new _54_LeftPut1RightScale2RightSwitch());
        AUTO_MODES.put(55, /* 	Drive Forward			 */ new _55_LeftDriveForward());
    }
	
		private static void initAutoModes2019() {
        AUTO_MODES_2019 = new HashMap<String, AutoModeBase>(); //THESE ARE FROM MARK'S "BIBLE"
        
        AUTO_MODES_2019.put("LeftRocketRearToFeedStationMode",  new LeftRocketRearToFeedStationMode());

        AUTO_MODES_2019.put("1",  new Mode_1());
        AUTO_MODES_2019.put("2",  new Mode_2());
        AUTO_MODES_2019.put("3",  new Mode_3());
        AUTO_MODES_2019.put("4",  new Mode_4());
        AUTO_MODES_2019.put("5",  new Mode_5());
        AUTO_MODES_2019.put("6",  new Mode_6());
        AUTO_MODES_2019.put("7",  new Mode_7());
        AUTO_MODES_2019.put("8",  new Mode_8());
        AUTO_MODES_2019.put("9",  new Mode_9());
        AUTO_MODES_2019.put("10", new Mode_10());
        AUTO_MODES_2019.put("A",  new Mode_A());
        AUTO_MODES_2019.put("B",  new Mode_B());
        AUTO_MODES_2019.put("C",  new Mode_C());
        AUTO_MODES_2019.put("D",  new Mode_D());
        AUTO_MODES_2019.put("E",  new Mode_E());
        AUTO_MODES_2019.put("F",  new Mode_F());
        AUTO_MODES_2019.put("G",  new Mode_G());
        AUTO_MODES_2019.put("H",  new Mode_H());
        AUTO_MODES_2019.put("I",  new Mode_I());
        AUTO_MODES_2019.put("J",  new Mode_J());
        
    }



                                        //   41 for example
	private AutoModeBase lookupMode(String autoCode) {
		AutoModeBase mode = null;
		if(autoCode != null && autoCode.length() > 0) {
			try {
				mode = AUTO_MODES.get(Integer.parseInt(autoCode));
			} catch (NumberFormatException e) {
				System.err.println("UNABLE TO PARSE DESIRED AUTO MODE!!!");
			}
		}
		return mode == null ? new StandStillMode() : mode;
	}

	private static void initAllowableAutoModes() {
		ALLOWABLE_AUTO_MODES = new HashMap<String, String[]>();// as defined in Mark's "BIBLE"
		ALLOWABLE_AUTO_MODES.put("LL", new String[]{"_00_DO_NOTHING",
													"_02_RightPut3LeftScale",
													"_03_RightPut2LeftScale1LeftSwitch",
													"_40_RightPut1LeftScale2LeftSwitch",
													"_05_RightDriveForward",
													"_35_MiddlePut2LeftSwitch",
													"_34_MiddlePut1LeftSwitch",
													"_36_MiddlePut1LeftSwitch1Exchange",
													"_47_MiddleDriveForward",
													"_31_LeftPut2LeftScale1LeftSwitch",
													"_23_LeftPut3LeftScale",
													"_48_LeftPut2LeftScaleEnd",
													"_49_LeftPut1LeftScaleEnd",
													"_50_LeftPut1LeftSwitchEnd1LeftScaleEnd",
													"_55_LeftDriveForward"
													});
		
		ALLOWABLE_AUTO_MODES.put("LR", new String[]{"_00_DO_NOTHING", 
													"_08_RightPut1RightScale1LeftSwitchEnd",
													"_07_RightPut3RightScale",
													"_09_RightPut2RightScale1LeftSwitch",
													"_41_RightPut1RightScaleEnd",
													"_42_RightPut1RightScaleEnd1LeftSwitch",
													"_43_RightPut2RightScaleEnd",
													"_05_RightDriveForward",
													"_34_MiddlePut1LeftSwitch",
													"_35_MiddlePut2LeftSwitch",
													"_47_MiddleDriveForward",
													"_27_LeftPut1LeftSwitchEnd1LeftSwitch",
													"_19_LeftPut3RightScale",
													"_28_LeftPut1LeftSwitchEnd1RightScale",
													"_55_LeftDriveForward"
													});
		
		ALLOWABLE_AUTO_MODES.put("RL", new String[]{"_00_DO_NOTHING", 
													"_11_RightPut1RightSwitchEnd1RightSwitch",
													"_02_RightPut3LeftScale",
													"_12_RightPut1RightSwitchEnd1LeftScale",
													"_05_RightDriveForward",
													"_37_MiddlePut1RightSwitch",
													"_38_MiddlePut2RightSwitch",
													"_39_MiddlePut1RightSwitch1Exchange",
													"_47_MiddleDriveForward",
													"_24_LeftPut1LeftScale1RightSwitchEnd",
													"_23_LeftPut3LeftScale",
													"_25_LeftPut2LeftScale1RightSwitch",
													"_49_LeftPut1LeftScaleEnd",
													"_52_LeftPut1LeftScaleEnd1RightSwitch",
													"_48_LeftPut2LeftScaleEnd",
													"_55_LeftDriveForward"
													});
		
		ALLOWABLE_AUTO_MODES.put("RR", new String[]{"_00_DO_NOTHING", 
													"_15_RightPut2RightScale1RightSwitch",
													"_07_RightPut3RightScale",
													"_43_RightPut2RightScaleEnd",
													"_41_RightPut1RightScaleEnd",
													"_46_RightPut1RightSwitchEnd1RightScaleEnd",
													"_05_RightDriveForward",
													"_37_MiddlePut1RightSwitch",
													"_38_MiddlePut2RightSwitch",
													"_39_MiddlePut1RightSwitch1Exchange",
													"_47_MiddleDriveForward",
													"_19_LeftPut3RightScale",
													"_20_LeftPut2RightScale1RightSwitch",
													"_54_LeftPut1RightScale2RightSwitch",
													"_55_LeftDriveForward"
													});
	}
	
	/**
     * This function is called periodically during autonomous
     */
    @Override
    public void autonomousPeriodic() {
        allPeriodic();
        
        //WPILIB WAY TO RUN AUTONOMOUS...
        //
        //
        //Scheduler.getInstance().run();
    }

    /**
     * Initializes the robot for the beginning of teleop
     */
    @Override
    public void teleopInit() {
        try {
            CrashTracker.logTeleopInit();

            // Start loopers
            mEnabledLooper.start();
            mDrive.setOpenLoop(DriveSignal.NEUTRAL);
            mDrive.setBrakeMode(false);
            // Shift to high
            mDrive.setHighGear(true);
            zeroAllSensors();
            mSuperstructure.reloadConstants();
            mSuperstructure.setOverrideCompressor(false);
			
			arduinoLedOutput(Constants.kArduino_TEAM);
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    /**
     * This function is called periodically during operator control.
     * 
     * The code uses state machines to ensure that no matter what buttons the driver presses, the robot behaves in a
     * safe and consistent manner.
     * 
     * Based on driver input, the code sets a desired state for each subsystem. Each subsystem will constantly compare
     * its desired and actual states and act to bring the two closer.
     */
    @Override
    public void teleopPeriodic() {
        try {
            double timestamp = Timer.getFPGATimestamp();
                
			//arduino.setcolor(mVisionCamProcessor.getVisionCamHasTarget() ? GREEN : RED);
            //SmartDashboard.putBoolean("visionCamHasTarget", mVisionCamProcessor.getVisionCamHasTarget());


            boolean climbUp = mControlBoard.getClimbUp();
            boolean climbDown = mControlBoard.getClimbDown();
            //boolean overTheTop = mControlBoard.getOverTheTopButton();
            boolean flipUp = mControlBoard.getFlipUpButton();
            boolean flipDown = mControlBoard.getFlipDownButton();
            boolean grabCube = mControlBoard.getGrabCubeButton();
            boolean calibrateDown = mControlBoard.getCalibrateDown();
            boolean calibrateUp = mControlBoard.getCalibrateUp();
            boolean spitting = mControlBoard.getSpit();
            boolean pickUp = mControlBoard.getAutoPickUp();
            
			
			//2019
			boolean inTractorBeam = false;
            // boolean grabCube = mControlBoard.getGrabCubeButton();
            // boolean calibrateDown = mControlBoard.getCalibrateDown();
            // boolean calibrateUp = mControlBoard.getCalibrateUp();
            // boolean spitting = mControlBoard.getSpit();
            // boolean pickUp = mControlBoard.getAutoPickUp();
            boolean pickupHatch = mControlBoard.getPickupPanel();
            boolean ejectHatch = mControlBoard.getShootPanel();
            boolean pickupCargo = mControlBoard.getPickupBall();
            boolean ejectCargo = mControlBoard.getShootBall();
            boolean elevCargoShipPos = mControlBoard.getCargoShipBall();
            boolean startingConfiguration = mControlBoard.getStartingConfiguration();       
            int climber = mControlBoard.getClimber();           



            if (climbUp) {
            	mSuperstructure.setWantedState(Superstructure.WantedState.CLIMBINGUP);
            } else if (climbDown) {
            	mSuperstructure.setWantedState(Superstructure.WantedState.CLIMBINGDOWN);
            } else if (grabCube) {
            	mSuperstructure.setWantedState(Superstructure.WantedState.INTAKING);
            } else if (spitting) {
            	mSuperstructure.setWantedState(Superstructure.WantedState.SPITTING);
            } else if (calibrateDown) {
            	mSuperstructure.setWantedState(Superstructure.WantedState.CALIBRATINGDOWN);
            } else if (calibrateUp) {
            	mSuperstructure.setWantedState(Superstructure.WantedState.CALIBRATINGUP);
				
				
			//2019
            } else if (startingConfiguration){
                mSuperstructure.setWantedState(Superstructure.WantedState.STARTINGCONFIGURATION);
            } else if (pickUp) {
                mSuperstructure.setWantedState(Superstructure.WantedState.AUTOINTAKING);
            } else if (ejectHatch) {
                mSuperstructure.setWantedState(Superstructure.WantedState.EJECTING_HATCH);
            } else if (pickupHatch) {
                mSuperstructure.setWantedState(Superstructure.WantedState.HATCH_CAPTURED); 
            } else if (ejectCargo) {
                mSuperstructure.setWantedState(Superstructure.WantedState.EJECTING_CARGO);
            } else if (pickupCargo) {
                mSuperstructure.setWantedState(Superstructure.WantedState.CARGO_CAPTURED);
			
				
				
            } else if (pickUp) {
            	mSuperstructure.setWantedState(Superstructure.WantedState.AUTOINTAKING);
            } else {
            	mSuperstructure.setWantedState(Superstructure.WantedState.ELEVATOR_TRACKING);
            }
            	
			//2018
            // if (flipUp) {
            //     mSuperstructure.setOverTheTop(GRABBER_POSITION.FLIP_UP);
            // } else if (flipDown) {
            //     mSuperstructure.setOverTheTop(GRABBER_POSITION.FLIP_DOWN);
            // } else {
            //     mSuperstructure.setOverTheTop(GRABBER_POSITION.FLIP_NONE);
            // }

			//[]TODO: modify for 2019 swerve
            // Drive base
            double throttle = mControlBoard.getThrottle();
            double turn = mControlBoard.getTurn();
            
            mDrive.setOpenLoop(mCheesyDriveHelper.cheesyDrive(throttle, turn, mControlBoard.getQuickTurn(),
                    !mControlBoard.getLowGear()));
            //boolean wantLowGear = mControlBoard.getLowGear();
            //mDrive.setHighGear(!wantLowGear);
            
            //2019
			if(mControlBoard.getInvertDrive()){
                joystickAxesAreReversed = !joystickAxesAreReversed;
                //toggleCamera(); 
            }
            if(mControlBoard.getToggleDriveSpeed()){
                driveSpeedIsToggled = !driveSpeedIsToggled;
                //toggleCamera(); 
            }
            if(joystickAxesAreReversed){
                throttle=-throttle;
                //leftRightCameraControl.set(true);
            }
            else{     
                //leftRightCameraControl.set(false);
            }


			////////////////////  2019 //////////////////////////////
			////////////////////  2019 //////////////////////////////
			////////////////////  2019 //////////////////////////////
			////////////////////  2019 //////////////////////////////
			////////////////////  2019 //////////////////////////////
            Optional<ShooterAimingParameters> aimParams;
            double now = Timer.getFPGATimestamp();
            aimParams = mRobotState.getAimingParameters();
            mDrive.setAimingParams(aimParams);

            if (mControlBoard.getTractorDrivePickupHatch() || mControlBoard.getTractorDriveEjectHatch())    
            { 

                if (mControlBoard.getTractorDrivePickupHatch()) {

                    if (!mTractorBeamPickupSelected) { // this is the first time in
                        mSuperstructure.prepareToPickupHatch();
                        mTractorBeamPickupSelected = true;
                    }

                    if  (aimParams.isPresent() && Math.abs(now - aimParams.get().getLastSeenTimestamp()) < 0.5  ) { 
                    mDrive.setWantTractorBeam();
                    }

                    if (mDrive.isTBFinished()) {
                        mSuperstructure.openBeak();
                    }
                }

                if (mControlBoard.getTractorDriveEjectHatch()) {
                    // System.out.println("im here!!!!!!!");
                    
                    if  (aimParams.isPresent() && Math.abs(now - aimParams.get().getLastSeenTimestamp()) < 0.5  ) {
                        mDrive.setWantTractorBeam();
                        }

                    if (mDrive.isTBFinished()) {
                     //+   mSuperstructure.ejectHatch();
                    }
                }
              //  System.out.println("A");
                if ((mDrive.isFirstTimeInTractorBeam()) &&   (aimParams.isPresent() && Math.abs(now - aimParams.get().getLastSeenTimestamp()) < 0.5  )) { 
               //     System.out.println("B");
                    mDrive.setIsDrivingTractorBeam(true);
                }
            } else if (!mControlBoard.getAutoLevel1ToCargoL1() &&           //5
                         !mControlBoard.getAutoCargoL1ToFeederStation() &&  //6
                         !mControlBoard.getAutoFeederStationToFront() &&    //4
                         !mControlBoard.getAutoFeederStationToRear() &&     //2
                         !mControlBoard.getAutoFrontToFeederStation()&&     //3
                         !mControlBoard.getAutoRearToFeederStation() &&     //1
                         !mControlBoard.getAutoLevel1ToRear()) {            //7
                mDrive.resetTractorBeam();
                if (mTractorBeamPickupSelected) {
                    mTractorBeamPickupSelected = false;
                    mSuperstructure.openBeak();

                }
            }

            if(mControlBoard.getAutoRearToFeederStation()){ //1
                if(mAutoModeExecuter == null){
                    mAutoModeExecuter = new AutoModeExecuter();
                    mAutoModeExecuter.setAutoMode( newLeftRocketRearToFeedStationMode );
                    mAutoModeExecuter.start();    
                }
            }
            else if(mControlBoard.getAutoFeederStationToRear()){ //2
                if(mAutoModeExecuter == null){
                    mAutoModeExecuter = new AutoModeExecuter();
                    //mAutoModeExecuter.setAutoMode( newLeftFeedStationToRocketRearMode );
                    mAutoModeExecuter.start();
                }
            }
            else if(mControlBoard.getAutoFrontToFeederStation()){ //3
                if(mAutoModeExecuter == null){
                    mAutoModeExecuter = new AutoModeExecuter();
                    //mAutoModeExecuter.setAutoMode( newLeftRocketFrontToFeedStationMode );
                    mAutoModeExecuter.start();
                }
            }
            else if(mControlBoard.getAutoFeederStationToFront()){ //4
                if(mAutoModeExecuter == null){
                    mAutoModeExecuter = new AutoModeExecuter();
                    //mAutoModeExecuter.setAutoMode( newLeftFeedStationToRocketFrontMode );
                    mAutoModeExecuter.start();
                }
            }
            else if(mControlBoard.getAutoLevel1ToCargoL1()){ //5
                if(mAutoModeExecuter == null){
                    mAutoModeExecuter = new AutoModeExecuter();
                    //mAutoModeExecuter.setAutoMode( newLeftLevel1ToCargoL1Mode );
                    mAutoModeExecuter.start();
                }
            }          
             else if(mControlBoard.getAutoCargoL1ToFeederStation()){ //6
                if(mAutoModeExecuter == null){
                    mAutoModeExecuter = new AutoModeExecuter();
                    //mAutoModeExecuter.setAutoMode( newLeftCargoL1ToFeederStationMode );
                    mAutoModeExecuter.start();
                }
            }

            else if(mControlBoard.getAutoLevel1ToRear()){  //7
                if(mAutoModeExecuter == null){
                    mAutoModeExecuter = new AutoModeExecuter();
                    //mAutoModeExecuter.setAutoMode( newLeftLevel1ToRocketRearMode );
                    mAutoModeExecuter.start();
                }
            }


            else if((climber != 1) && !mDrive.isDrivingTractorBeam()){
                stopAuto(); // if none of the above 4 auto buttons is being held down and we're not climbing

                //regular cheesy drive
                if(driveSpeedIsToggled){
                    throttle *= 0.5;//'slow' driver controller speed
                    turn *= 0.3;
                    arduinoLedOutput(Constants.kArduino_YELLW);
                }
                else{
                    throttle *= 0.9;//'normal' driver controller speed
                    turn *= 0.5;
                    arduinoLedOutput(Constants.kArduino_BLUEW);
                }
                mDrive.setOpenLoop(mCheesyDriveHelper.cheesyDrive(throttle, turn, mControlBoard.getQuickTurn(),
                                                                    !mControlBoard.getLowGear()));
                boolean wantLowGear = mControlBoard.getLowGear();
                mDrive.setHighGear(!wantLowGear);
                mClimber.setWantedState(Climber.WantedState.IDLE);

                if(mRobotState.getFieldToVehicle(timestamp).getTranslation().x() > (Constants.FIELD_CTR_INCHES - Constants.WARNING_BUFFER_INCHES)){
               //     mControlBoard.rumbleDriver();
                }
            }


			////////////////////  2019 //////////////////////////////
			////////////////////  2019 //////////////////////////////
			////////////////////  2019 //////////////////////////////
			////////////////////  2019 //////////////////////////////
			////////////////////  2019 //////////////////////////////
			////////////////////  2019 //////////////////////////////

            allPeriodic();
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    private void arduinoLedOutput(int value) {
        arduinoLed0.set((value & 0x01)==0 ? Boolean.FALSE: Boolean.TRUE);
        arduinoLed1.set((value & 0x02)==0 ? Boolean.FALSE: Boolean.TRUE);
        arduinoLed2.set((value & 0x04)==0 ? Boolean.FALSE: Boolean.TRUE);
    }

    private void stopAuto(){
        if(mAutoModeExecuter != null){
            mAutoModeExecuter.stop();
            mAutoModeExecuter = null;
        }
    }

    @Override
    public void disabledInit() {
        try {
            CrashTracker.logDisabledInit();

            if (mAutoModeExecuter != null) {
                mAutoModeExecuter.stop();
            }
            mAutoModeExecuter = null;

            mEnabledLooper.stop();

            // Call stop on all our Subsystems.
            mSubsystemManager.stop();

            mDrive.setOpenLoop(DriveSignal.NEUTRAL);

            PathAdapter.calculatePaths();

            // If are tuning, dump map so far.
            if (Constants.kIsShooterTuning) {
                for (Map.Entry<InterpolatingDouble, InterpolatingDouble> entry : mTuningFlywheelMap.entrySet()) {
                    System.out.println("{" +
                            entry.getKey().value + ", " + entry.getValue().value + "},");
                }
            }
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void disabledPeriodic() {
        final double kVoltageThreshold = 0.15;
        if (mCheckLightButton.getAverageVoltage() < kVoltageThreshold) {
            //mLED.setLEDOn();
        } else {
            //mLED.setLEDOff();
        }

		//2018
        zeroAllSensors();
		
		//2019
        autoCode = SmartDashboard.getString("AutoCode", autoCode); // or R
        Optional<ShooterAimingParameters> aimParams;
        double now = Timer.getFPGATimestamp();
        aimParams = mRobotState.getAimingParameters();
        mDrive.setAimingParams(aimParams);    
        greenLEDRingLight.set(false); // turn off the light until teleop
		
		
        allPeriodic();
    }

    @Override
    public void testInit() {
        Timer.delay(0.5);

        boolean results = Elevator.getInstance().checkSystem();
        results &= Drive.getInstance().checkSystem();
        results &= Intake.getInstance().checkSystem();


        if (!results) {
            System.out.println("CHECK ABOVE OUTPUT SOME SYSTEMS FAILED!!!");
        } else {
            System.out.println("ALL SYSTEMS PASSED");
        }
    }

    @Override
    public void testPeriodic() {
    }

    /**
     * Helper function that is called in all periodic functions
     */
    public void allPeriodic() {
        mRobotState.outputToSmartDashboard();
        mSubsystemManager.outputToSmartDashboard();
        mSubsystemManager.writeToLog();
        mEnabledLooper.outputToSmartDashboard();
        SmartDashboard.putString("AutoCode", autoCode);
        SmartDashboard.putBoolean("Tractor Beam", tractorIndicator);

        ConnectionMonitor.getInstance().setLastPacketTime(Timer.getFPGATimestamp());
    }
}
