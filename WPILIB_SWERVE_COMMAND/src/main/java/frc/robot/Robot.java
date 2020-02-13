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
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TargetingSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.util.DebugOutput;
import frc.robot.util.ReflectingCSVWriter;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  private ReflectingCSVWriter<DebugOutput> mCSVWriter;
  private AnalogInput leftFrontAbsEncoder;
  private AnalogInput rightFrontAbsEncoder;
  private AnalogInput leftRearAbsEncoder;
  private AnalogInput rightRearAbsEncoder;
  
  // The robot's subsystems
  public DriveSubsystem m_robotDrive;
  public IntakeSubsystem m_intake;
  public ShooterSubsystem m_shooter;
  public TargetingSubsystem m_targeting;
  public VisionSubsystem m_vision;
  public ClimberSubsystem m_climber;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    if(RobotBase.isReal()){
      mCSVWriter = new ReflectingCSVWriter<DebugOutput>("/home/lvuser/PATH-FOLLOWER-LOGS.csv", DebugOutput.class);
    }
    else{
      mCSVWriter = new ReflectingCSVWriter<DebugOutput>("PATH-FOLLOWER-LOGS.csv", DebugOutput.class);
    }

    m_robotDrive = new DriveSubsystem(mCSVWriter);
    m_intake = new IntakeSubsystem(mCSVWriter);
    m_shooter = new ShooterSubsystem(mCSVWriter);
    m_targeting = new TargetingSubsystem(mCSVWriter);
    m_vision = new VisionSubsystem(mCSVWriter);
    m_climber = new ClimberSubsystem(mCSVWriter);
    
    m_robotDrive.zeroHeading();

    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer(m_robotDrive, m_intake, m_shooter, m_targeting, m_vision, m_climber);


    leftFrontAbsEncoder = new AnalogInput(1);
    rightFrontAbsEncoder = new AnalogInput(2);
    leftRearAbsEncoder = new AnalogInput(3);
    rightRearAbsEncoder = new AnalogInput(4);

    m_robotDrive.resetEncoders(leftFrontAbsEncoder.getVoltage(),
                               rightFrontAbsEncoder.getVoltage(),
                               leftRearAbsEncoder.getVoltage(),
                               rightRearAbsEncoder.getVoltage());


    SmartDashboard.putString("Auto Num", "0");
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {

    if(mCSVWriter.isSuspended()){
      mCSVWriter.resume();
    }

    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
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
    if(!mCSVWriter.isSuspended()){
      mCSVWriter.suspend();
    }
  }

  @Override
  public void disabledPeriodic() {

  }

  /**
   * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    if(mCSVWriter.isSuspended()){
      mCSVWriter.resume();
    }


    int autoNum = 0;
    String autoSelected = "0"; //DEFAULT AUTO MODE if Drive Team forgets
    if(RobotBase.isReal()){
      autoSelected = SmartDashboard.getString("Auto Selector", "2");
    }
    try{
      autoNum = Math.abs(Integer.parseInt(autoSelected));
    }
    catch(Exception e){
      System.out.println("AUTO NUM did not parse -- defaulting to MOVE FORWARD!!!");
    }
    NamedAutoCommand namedAutoCommand = m_robotContainer.getNamedAutonomousCommand(autoNum);
    m_autonomousCommand = namedAutoCommand.command;

    // schedule the autonomous command (example)
    System.out.println("Running auto mode " + namedAutoCommand.name);
    m_autonomousCommand.schedule();
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    if(mCSVWriter.isSuspended()){
      mCSVWriter.resume();
    }

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
