/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ColorWheelSubsystem;
import frc.robot.subsystems.LedStringSubsystem;
import frc.robot.subsystems.SequencerSubsystem;
import frc.robot.subsystems.ShootClimbSubsystem;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  // The robot's subsystems and commands are defined here...
  public IntakeSubsystem m_intake = new IntakeSubsystem();
  public SequencerSubsystem m_sequencer = new SequencerSubsystem();
  public ShootClimbSubsystem m_shootclimb = new ShootClimbSubsystem();
  public ColorWheelSubsystem m_colorwheel = new ColorWheelSubsystem();
  public LedStringSubsystem m_ledstring = new LedStringSubsystem();

  
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer(m_intake, m_sequencer, m_shootclimb);
    
    m_robotContainer.initSmartDashboard();

  // initial SubSystems to at rest states
    m_intake.retract();
    m_sequencer.stop();
    m_shootclimb.disable();
    m_colorwheel.init();
    m_ledstring.init();
  
        
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
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    
    int autoNum = 0;
    String autoSelected = SmartDashboard.getString("Auto Selector", "Default");
    try{
      autoNum = Integer.parseInt(autoSelected);
    }
    catch(Exception e){
      System.out.println("AUTO NUM did not parse -- default to DO NOTHING!!!!");
    }
    System.out.println("Running auto mode " + autoNum);
    m_autonomousCommand = m_robotContainer.getAutonomousCommand(autoNum);

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
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
    m_robotContainer.outputToSmartDashboard();
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
