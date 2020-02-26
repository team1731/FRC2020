/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.JevoisVisionSubsystem;
import frc.robot.Constants.OpConstants.LedOption;
import frc.robot.autonomous._NamedAutoMode;
import frc.robot.subsystems.ColorWheelSubsystem;
import frc.robot.subsystems.LedStringSubsystem;
import frc.robot.subsystems.SequencerSubsystem;
import frc.robot.subsystems.ShootClimbSubsystem;
import frc.robot.vision.JevoisVisionServer;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;

  // The robot's subsystems
  public DriveSubsystem m_robotDrive;
  public JevoisVisionSubsystem m_vision;
  public IntakeSubsystem m_intake;
  public SequencerSubsystem m_sequencer;
  public ShootClimbSubsystem m_shootclimb;
  public ColorWheelSubsystem m_colorwheel;
  public LedStringSubsystem m_ledstring;

  private void initSubsystems(){
    // initial SubSystems to at rest states
    m_robotDrive.resetEncoders();
    m_intake.retract();
    m_sequencer.stop();
    m_shootclimb.stopShooting();
    //m_colorwheel.init();
    //m_ledstring.init();
  }

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    
    CameraServer camServer = CameraServer.getInstance();
    camServer.startAutomaticCapture();
    
    m_ledstring = null; //new LedStringSubsystem();
    m_robotDrive = new DriveSubsystem();
    m_vision = JevoisVisionSubsystem.getInstance();
    m_vision.setDriveSubsystem(m_robotDrive);
    m_intake = new IntakeSubsystem(m_ledstring);
    m_sequencer = new SequencerSubsystem(m_ledstring);
    m_shootclimb = new ShootClimbSubsystem(m_ledstring);
    m_colorwheel = null; //new ColorWheelSubsystem();

    m_robotDrive.zeroHeading();

    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer(m_ledstring, m_robotDrive, m_intake, m_sequencer, m_shootclimb, m_vision);


    initSubsystems();

    SmartDashboard.putString("INIT CELL COUNT", "3"); // How much ammo we start with

    SmartDashboard.putString("AUTO CODE", "T5"); // XNDD (X=L,M,R,F) (N=1,2,3,4) (DD=0-99 [optional])
                                                 // XN = one of Mark and Chuck's 10 auto modes plus new "forward" mode F
                                                 //      (and if it turns out we need a backward mode, B, we will add it)
                                                 // DD = up to 2 digits (0-9) signifying 2 possible delays (in seconds)
                                                 // 1st D = 0-9 second delay at very beginning of auto
                                                 // 2nd D = 0-9 second delay after first shooting event
                                                 // examples:
                                                 // M1 --> run M1 auto with NO DELAYS
                                                 // M25 --> wait 5 seconds, then run M2 auto
                                                 // M203 --> wait 0 seconds, run M2 with 3-sec delay after 1st shooting
                                                 // F12 --> wait 2 seconds, run "forward" auto mode (robot will drive forward a pre-programmed distance)
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods. This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

    m_robotDrive.displayEncoders();
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit() {
    //m_robotDrive.suspendCSVWriter();
    if(m_vision != null){
      //m_vision.StopCameraDataStream();
    }
  }

  @Override
  public void disabledPeriodic() {
    //m_ledstring.option(LedOption.TEAM);
    m_robotDrive.resetEncoders();
    SmartDashboard.putBoolean("LowSensor",  m_sequencer.lowSensorHasBall());
    SmartDashboard.putBoolean("MidSensor",  m_sequencer.midSensorHasBall());
    SmartDashboard.putBoolean("HighSensor",  m_sequencer.highSensorHasBall());
   }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    CommandScheduler.getInstance().cancelAll(); ///????????????????????????????????????????? SHOULD WE DO THIS????? ******************
    
    //m_ledstring.option(LedOption.RAINBOW);

    //m_robotDrive.resumeCSVWriter();
    m_sequencer.setPowerCellCount((int) SmartDashboard.getNumber("INIT CELL COUNT", 3));

    if(m_vision != null){
      //m_vision.StartCameraDataStream();
    }

    String DEFAULT_AUTO_CODE = "T5"; // DEFAULT AUTO MODE if Drive Team is unable to set the mode via Dashboard
                                     // NOTE: also useful if trying to run in the simulator!
    String autoCode = DEFAULT_AUTO_CODE;
    if (RobotBase.isReal()) {
      autoCode = SmartDashboard.getString("AUTO CODE", autoCode);
    }
    System.out.println("AUTO CODE retrieved from Dashboard --> " + autoCode);
    if (autoCode == null || autoCode.length() < 2) {
      autoCode = DEFAULT_AUTO_CODE;
    }
    autoCode = autoCode.toUpperCase();
    System.out.println("AUTO CODE being used by the software --> " + autoCode);

    m_autonomousCommand = null;
    _NamedAutoMode namedAutoMode = m_robotContainer.getNamedAutonomousCommand(autoCode);
    if(namedAutoMode != null){
      m_autonomousCommand = namedAutoMode.getCommand();

      // schedule the autonomous command (example)
      if (m_autonomousCommand == null) {
        System.err.println("SOMETHING WENT WRONG - UNABLE TO RUN AUTONOMOUS! CHECK SOFTWARE!");
      } else {
        System.out.println("Running actual autonomous mode --> " + namedAutoMode.name);
        m_autonomousCommand.schedule();
      }
    }
    else{
      System.err.println("UNABLE TO EXECUTE SELECTED AUTONOMOUS MODE!!");
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    CommandScheduler.getInstance().cancelAll();
    m_sequencer.setPowerCellCount((int) SmartDashboard.getNumber("CELL COUNT", 3));
    //m_robotDrive.resumeCSVWriter();

    initSubsystems();

    //m_vision.StartCameraDataStream();

    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

  }


  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    SmartDashboard.putBoolean("LowSensor",  m_sequencer.lowSensorHasBall());
    SmartDashboard.putBoolean("MidSensor",  m_sequencer.midSensorHasBall());
    SmartDashboard.putBoolean("HighSensor",  m_sequencer.highSensorHasBall());
    SmartDashboard.putNumber("PowerCellCount",  (int)m_sequencer.getPowerCellCount());
    SmartDashboard.putString("Intake State",  m_intake.getIntakeState());
    //SmartDashboard.putNumber("Climb Encoder", m_shootclimb.getClimbEncoderValue());

    // switch((int)m_sequencer.getPowerCellCount()){
    //   case 1: m_ledstring.option(LedOption.BALLONE); break;
    //   case 2: m_ledstring.option(LedOption.BALLTWO); break;
    //   case 3: m_ledstring.option(LedOption.BALLTHREE); break;
    //   case 4: m_ledstring.option(LedOption.BALLFOUR); break;
    //   case 5: m_ledstring.option(LedOption.GREEN); break;
    // }
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
    SmartDashboard.putNumber("Shoot Motor % (0-1)", 0.5);
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
    double shootMotorPercent_0_to_1 = SmartDashboard.getNumber("Shoot Motor % (0-1)", 0.5);
    m_shootclimb.hoodExtend();
    m_shootclimb.spinShooter(shootMotorPercent_0_to_1);
    //SmartDashboard.putNumber("Shoot Motor 1 Vel", m_shootclimb.getShootMotor1Velocity());
  }
}
