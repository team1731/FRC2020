package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.control.Controls;
import frc.robot.subsystem.DriveSubsystem;

import java.util.Arrays;
import java.util.Date;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import org.strykeforce.thirdcoast.swerve.Wheel;
import org.usfirst.frc.team1731.lib.util.CrashTracker;
import org.usfirst.frc.team1731.lib.util.math.RigidTransform2d;
import org.usfirst.frc.team1731.robot.SubsystemManager;
import org.usfirst.frc.team1731.robot.auto.AutoModeExecuter;
import org.usfirst.frc.team1731.robot.subsystems.Climber;
import org.usfirst.frc.team1731.robot.subsystems.Drive;
import org.usfirst.frc.team1731.robot.subsystems.Intake;
import org.usfirst.frc.team1731.robot.subsystems.Superstructure;
import org.usfirst.frc.team1731.robot.RobotState;


public class Robot extends TimedRobot {
  private static final Logger logger = LoggerFactory.getLogger(Robot.class);
  public static final DriveSubsystem DRIVE = new DriveSubsystem();

  // Controls initialize Commands so this should be instantiated last to prevent
  // NullPointerExceptions in commands that require() Subsystems above.
  public static final Controls CONTROLS = new Controls();

  private CANSparkMax m_motor;
  private CANPIDController m_pidController;
  private CANEncoder m_encoder;
  private static final int deviceID = 1;
  private Wheel[] wheelObjects;

  //#region ROBOTNEW

  private AutoModeExecuter mAutoModeExecuter = null;
  private Superstructure mSuperstructure = Superstructure.getInstance();
  private RobotState mRobotState = RobotState.getInstance();

  private final SubsystemManager mSubsystemManager = new SubsystemManager(
                           Arrays.asList(
                                         Drive.getInstance(),
                                         Superstructure.getInstance(),
                                         //Elevator.getInstance(),
                                         Intake.getInstance(),
                                         Climber.getInstance()
                                         ));

    public Robot() {
      //2019
      super(0.2);
      
      CrashTracker.logRobotConstruction();
    }
    //#endregion



  @Override
  public void robotInit() {
    //logger.info("<b>Robot</b>: robotInit Started");
    System.out.println("Today is " + new Date().toString());
    DRIVE.zeroAzimuthEncoders();
    DRIVE.zeroGyro();

    //logger.info("<b>Robot</b>: robotInit Finished");
    //just some example lines i used to prove that code completion still works!
    //m_motor = new CANSparkMax(deviceID, MotorType.kBrushless);
    //m_motor.disable();
  }

  @Override
  public void teleopPeriodic() {
    //logger.info("<b>Robot</b>: teleopPeriodic started");
    Scheduler.getInstance().run();
    //logger.info("<b>Robot</b>: teleopPeriodic finished");
  }

  @Override
  public void disabledPeriodic(){
    if(wheelObjects == null){
      wheelObjects = DRIVE.getWheelObjects();
    } else if(wheelObjects.length == 4) {
      for (int i = 0; i < wheelObjects.length; i++) {
        Wheel wheel = wheelObjects[i];
        if(wheel.m_encoder != null){
          SmartDashboard.putNumber("Wheel"+wheel.wheelID+" Encoder Pos", wheel.m_encoder.getPosition());
          SmartDashboard.putNumber("Wheel"+wheel.wheelID+" Encoder Get Abs", wheel.getAzimuthAbsolutePosition());
        }
      }
    }
  }

  //#region ROBOTNEW

   /**
     * Initializes the robot for the beginning of autonomous mode (set drivebase, intake and superstructure to correct
     * states). Then gets the correct auto mode from the AutoModeSelector
     * 
     * @see AutoModeSelector.java
     */
    /*
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
        } catch(Exception e) {
            //aaaaaaaaa scary
        }
    }

    public void zeroAllSensors() {
      mSubsystemManager.zeroSensors();
      mRobotState.reset(Timer.getFPGATimestamp(), new RigidTransform2d());
     // mDrive.zeroSensors(); subsystem manager does this
    }
*/
    //#endregion
}
