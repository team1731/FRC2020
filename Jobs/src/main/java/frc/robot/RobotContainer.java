/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
//import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SequencerSubsystem;
import frc.robot.subsystems.ShootClimbSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.DigitalInput;

import frc.robot.commands.IntakeEject;


/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
  private final SequencerSubsystem m_SequencerSubsystem = new SequencerSubsystem();
  private final ShootClimbSubsystem m_ShootClimbSubsystem = new ShootClimbSubsystem();

  //private DigitalInput m_LowSensor = new DigitalInput(Constants.kLowSequencer);
  //private final ExampleCommand m_autoCommand = new ExampleCommand(m_IntakeSubsystem);


  XboxController driverController = new XboxController(0); 
  XboxController operatorController = new XboxController(1); 

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    //m_IntakeSubsystem.setDefaultCommand(new InstantCommand(m_IntakeSubsystem::retract));
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Intake
    new JoystickButton(operatorController, 1)
        .whenPressed(new InstantCommand(m_IntakeSubsystem::extend, m_IntakeSubsystem))
        .whenReleased(new InstantCommand(m_IntakeSubsystem::retract, m_IntakeSubsystem));
    new JoystickButton(operatorController, 2)
        .whenPressed(new IntakeEject(m_IntakeSubsystem))
        .whenReleased(new InstantCommand(m_IntakeSubsystem::retract, m_IntakeSubsystem));
        //.whenPressed(new InstantCommand(m_IntakeSubsystem::eject, m_IntakeSubsystem))
        //.whenReleased(new InstantCommand(m_IntakeSubsystem::retract, m_IntakeSubsystem));

    // Sequencer
    new JoystickButton(operatorController, 3)
        .whenPressed(new InstantCommand(m_SequencerSubsystem::forward, m_IntakeSubsystem))
        .whenReleased(new InstantCommand(m_SequencerSubsystem::stop, m_IntakeSubsystem));
    new JoystickButton(operatorController, 4)
        .whenPressed(new InstantCommand(m_SequencerSubsystem::reverse))
        .whenReleased(new InstantCommand(m_SequencerSubsystem::stop));

    /*/ conditional Sequencer when Intake is pushed
    new JoystickButton(operatorController, 0)
      .whenPressed(new ParallelCommandGroup(
          new InstantCommand(m_IntakeSubsystem::extend, m_IntakeSubsystem),
          new ConditionalCommand(
            new InstantCommand(m_SequencerSubsystem::addBall, m_SequencerSubsystem),
            new InstantCommand(m_SequencerSubsystem::stop, m_SequencerSubsystem),
            m_LowSensor::get)
          )
        )
      .whenReleased(new ParallelCommandGroup(
          new InstantCommand(m_IntakeSubsystem::retract, m_IntakeSubsystem),
          new InstantCommand(m_SequencerSubsystem::stop, m_SequencerSubsystem)
        )
      );
  */
    // Shoot
    new JoystickButton(operatorController, 5)
        .whenPressed(new InstantCommand(m_ShootClimbSubsystem::on, m_ShootClimbSubsystem))
        .whenReleased(new InstantCommand(m_ShootClimbSubsystem::off, m_ShootClimbSubsystem));
    new JoystickButton(operatorController, 6)
        .whenPressed(new InstantCommand(m_ShootClimbSubsystem::on, m_ShootClimbSubsystem))
        .whenReleased(new InstantCommand(m_ShootClimbSubsystem::off, m_ShootClimbSubsystem));
    
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  /*
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }
  */
}
