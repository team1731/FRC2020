/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.JevoisVisionSubsystem;
import frc.robot.autonomous._NamedAutoMode;
import frc.robot.subsystems.ColorWheelSubsystem;
import frc.robot.subsystems.LedStringSubsystem;
import frc.robot.subsystems.SequencerSubsystem;
import frc.robot.subsystems.ShootClimbSubsystem;
import frc.robot.subsystems.TargetingSubsystem;
import frc.robot.util.DebugOutput;
import frc.robot.util.ReflectingCSVWriter;
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
  private AnalogInput leftFrontAbsEncoder;
  private AnalogInput rightFrontAbsEncoder;
  private AnalogInput leftRearAbsEncoder;
  private AnalogInput rightRearAbsEncoder;

  // The robot's subsystems
  public DriveSubsystem m_robotDrive;
  public TargetingSubsystem m_targeting;
  public JevoisVisionSubsystem m_vision;
  public IntakeSubsystem m_intake;
  public SequencerSubsystem m_sequencer;
  public ShootClimbSubsystem m_shootclimb;
  public ColorWheelSubsystem m_colorwheel;
  public LedStringSubsystem m_ledstring;

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    JevoisVisionServer.getInstance();

    m_robotDrive = new DriveSubsystem();
    m_targeting = new TargetingSubsystem();
    m_vision = new JevoisVisionSubsystem();
    m_intake = new IntakeSubsystem();
    m_sequencer = new SequencerSubsystem();
    m_shootclimb = new ShootClimbSubsystem();
    m_colorwheel = new ColorWheelSubsystem();
    m_ledstring = new LedStringSubsystem();

    m_robotDrive.zeroHeading();

    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer(m_robotDrive, m_intake, m_sequencer, m_shootclimb, m_targeting, m_vision);

    leftFrontAbsEncoder = new AnalogInput(1);
    rightFrontAbsEncoder = new AnalogInput(2);
    leftRearAbsEncoder = new AnalogInput(3);
    rightRearAbsEncoder = new AnalogInput(4);

    m_robotDrive.resetEncoders(leftFrontAbsEncoder.getVoltage(), rightFrontAbsEncoder.getVoltage(),
        leftRearAbsEncoder.getVoltage(), rightRearAbsEncoder.getVoltage());

    // initial SubSystems to at rest states
    m_intake.retract();
    m_sequencer.stop();
    m_shootclimb.disable();
    m_colorwheel.init();
    m_ledstring.init();

    SmartDashboard.putString("Auto Code", "M1"); // XNDD (X=L,M,R,F) (N=1,2,3,4) (DD=0-99 [optional])
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
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

    SmartDashboard.putNumber("leftFrontAbsEncoder", leftFrontAbsEncoder.getVoltage()); // 0.0 to 3.26, 180=1.63V
    SmartDashboard.putNumber("rightFrontAbsEncoder", rightFrontAbsEncoder.getVoltage()); // 0.0 to 3.26, 180=1.63V
    SmartDashboard.putNumber("leftRearAbsEncoder", leftRearAbsEncoder.getVoltage()); // 0.0 to 3.26, 180=1.63V
    SmartDashboard.putNumber("rightRearAbsEncoder", rightRearAbsEncoder.getVoltage()); // 0.0 to 3.26, 180=1.63V
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit() {
    m_robotDrive.suspendCSVWriter();
  }

  @Override
  public void disabledPeriodic() {

   }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    m_robotDrive.resumeCSVWriter();

    String DEFAULT_AUTO_CODE = "F1"; // DEFAULT AUTO MODE if Drive Team is unable to set the mode via Dashboard
    String autoCode = DEFAULT_AUTO_CODE;
    if (RobotBase.isReal()) {
      autoCode = SmartDashboard.getString("Auto Code", autoCode);
    }
    System.out.println("Auto Code retrieved from Dashboard --> " + autoCode);
    if(autoCode == null || autoCode.length() < 2){
      autoCode = DEFAULT_AUTO_CODE;
    }
    autoCode = autoCode.toUpperCase();
    System.out.println("Auto Code being used by the software --> " + autoCode);
    _NamedAutoMode namedAutoCommand = m_robotContainer.getNamedAutonomousCommand(autoCode);
    m_autonomousCommand = namedAutoCommand.getCommand();

    // schedule the autonomous command (example)
    if(m_autonomousCommand == null){
      System.out.println("SOMETHING WENT WRONG - UNABLE TO RUN AUTONOMOUS! CHECK SOFTWARE!");
    }
    else{
      System.out.println("Running actual autonomous mode --> " + namedAutoCommand.name);
      m_autonomousCommand.schedule();
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
    m_robotDrive.resumeCSVWriter();

    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    SmartDashboard.putBoolean("LowSensor",  m_sequencer.getLowSensor());
    SmartDashboard.putNumber("PowerCellCount",  (double)m_sequencer.getPowerCellCount());
    SmartDashboard.putString("Intake State",  m_intake.getIntakeState());
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {

  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
