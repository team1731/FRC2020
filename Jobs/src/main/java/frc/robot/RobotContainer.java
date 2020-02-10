/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;
import edu.wpi.first.wpilibj.GenericHID.Hand;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;

import frc.robot.commands.*;
import frc.robot.commands.IntakeRetract;
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

  ShuffleboardTab sensorTab;

  // The robot's subsystems and commands are defined here...
  private final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
  private final SequencerSubsystem m_SequencerSubsystem = new SequencerSubsystem();
  private final ShootClimbSubsystem m_ShootClimbSubsystem = new ShootClimbSubsystem();

  //private DigitalInput m_LowSensor = new DigitalInput(Constants.kLowSequencer);
  //private final ExampleCommand m_autoCommand = new ExampleCommand(m_IntakeSubsystem);


  XboxController driverController = new XboxController(0); 
  XboxController operatorController = new XboxController(1); 
  private NetworkTableEntry eLowSensor;
  private NetworkTableEntry ePowerCellCount;
  private NetworkTableEntry eIntakeState;
  private double axis3;
  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    /*
    m_IntakeSubsystem.setDefaultCommand(
      new IntakeSeqCommand(m_IntakeSubsystem, m_SequencerSubsystem, () -> getIntake())
    );
    
    m_ShootClimbSubsystem.setDefaultCommand(
      new ShootClimbCommand(m_ShootClimbSubsystem, m_SequencerSubsystem, () -> getShoot())
    );
    */
    /*
    //m_IntakeSubsystem.setDefaultCommand(new InstantCommand(m_IntakeSubsystem::retract));
    m_IntakeSubsystem.setDefaultCommand(
      //new IntakeRetract(m_IntakeSubsystem, () -> getIntake())
      new IntakeRetract(m_IntakeSubsystem)
    );

    //m_SequencerSubsystem.setDefaultCommand(new IntakeRetract(m_IntakeSubsystem), () -> getIntake());
    m_SequencerSubsystem.setDefaultCommand(
      //new IntakeRetract(m_IntakeSubsystem, () -> getIntake())
      new InstantCommand(m_SequencerSubsystem::stop)
    );
    */
  }

  public void initSubsystems() {
    m_IntakeSubsystem.retract();
    m_SequencerSubsystem.stop();
    m_ShootClimbSubsystem.disable();
  }
  
  public boolean getIntake() {
    double n = operatorController.getTriggerAxis(Hand.kLeft);
    return Math.abs(n) > 0.5;
  }

  public boolean getShoot() {
    double n = operatorController.getTriggerAxis(Hand.kRight);
    return Math.abs(n) > 0.5;
  }
  
  //public double getIntake() {
  //  double n = operatorController.getTriggerAxis(Hand.kLeft);
  //  return Math.abs(n) < 0.1 ? 0 : n;
  //}

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    // Intake & Sequencer ejects works will button is held
    new JoystickButton(operatorController, 1).whenHeld(new ParallelCommandGroup(
      new InstantCommand(m_IntakeSubsystem::eject, m_IntakeSubsystem),
      new InstantCommand(m_SequencerSubsystem::reverse, m_SequencerSubsystem)
      )).whenReleased(new ParallelCommandGroup(
        new InstantCommand(m_IntakeSubsystem::retract, m_IntakeSubsystem),
        new InstantCommand(m_SequencerSubsystem::stop, m_SequencerSubsystem)
      )
    );

    // Intake & Sequencer intake works will button is held
    new JoystickButton(operatorController, 5).whenHeld(
      new IntakeSeqCommand(m_IntakeSubsystem, m_SequencerSubsystem)
    );
    new IntakeTrigger().whileActiveOnce(new IntakeSeqCommand(m_IntakeSubsystem, m_SequencerSubsystem));

    // Shooter
    new JoystickButton(operatorController, 6).whenHeld(
      new ShootSeqCommand(m_ShootClimbSubsystem, m_SequencerSubsystem)
    );
    new ShootTrigger().whileActiveOnce(new ShootSeqCommand(m_ShootClimbSubsystem, m_SequencerSubsystem));
    /* Intake
    new JoystickButton(operatorController, 1)
        .whenPressed(new InstantCommand(m_IntakeSubsystem::extend, m_IntakeSubsystem));
        //.whenReleased(new InstantCommand(m_IntakeSubsystem::retract, m_IntakeSubsystem));
    new JoystickButton(operatorController, 2)
        .whenPressed(new IntakeEject(m_IntakeSubsystem));
        //.whenReleased(new InstantCommand(m_IntakeSubsystem::retract, m_IntakeSubsystem));
        //.whenPressed(new InstantCommand(m_IntakeSubsystem::eject, m_IntakeSubsystem))
        //.whenReleased(new InstantCommand(m_IntakeSubsystem::retract, m_IntakeSubsystem));
    */
    /* Sequencer
    new JoystickButton(operatorController, 3)
        .whenPressed(new InstantCommand(m_SequencerSubsystem::forward, m_IntakeSubsystem))
        .whenReleased(new InstantCommand(m_SequencerSubsystem::stop, m_IntakeSubsystem));
    new JoystickButton(operatorController, 4)
        .whenPressed(new InstantCommand(m_SequencerSubsystem::reverse))
        .whenReleased(new InstantCommand(m_SequencerSubsystem::stop));
    */
    // Shoot
    new JoystickButton(operatorController, 3)
        .whenPressed(new InstantCommand(m_ShootClimbSubsystem::modeClimb, m_ShootClimbSubsystem));
        //.whenReleased(new InstantCommand(m_ShootClimbSubsystem::off, m_ShootClimbSubsystem));
    new JoystickButton(operatorController, 4)
        //.whenPressed(new InstantCommand(m_ShootClimbSubsystem::on, m_ShootClimbSubsystem))
        .whenReleased(new InstantCommand(m_ShootClimbSubsystem::modeShoot, m_ShootClimbSubsystem));
    
    //new JoystickButton(operatorController, 7)
    //  .whenHeld(new AutoIntakeSeqCommand(m_IntakeSubsystem, m_SequencerSubsystem));
      //.whenInactive(new IntakeRetract(m_IntakeSubsystem));

    //new JoystickButton(operatorController, XboxController.Axis.kLeftTrigger).whenHeld(command)

    //new JoystickButton(operatorController, 7)
    //  .whenActive(new AutoIntakeSeqCommand(m_IntakeSubsystem, m_SequencerSubsystem))
    //  .whenInactive(new IntakeRetract(m_IntakeSubsystem));

      //axis3 = operatorController.getX(Hand.kLeft);
    //axis3 = operatorController.getTriggerAxis(Hand.kRight); // new Trigger(operatorController, 1);
    
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

  public class IntakeTrigger extends Trigger {
    @Override
    public boolean get() {
      return getIntake();
    }
  }

  public class ShootTrigger extends Trigger {
    @Override
    public boolean get() {
      return getShoot();
    }
  }

  public void initSmartDashboard() {
    sensorTab = Shuffleboard.getTab("Sensors");
    eLowSensor = sensorTab.add("LowSensor", m_SequencerSubsystem.getLowSensor()).getEntry();
    ePowerCellCount =sensorTab.add("PowerCellCount", m_SequencerSubsystem.getPowerCellCount()).getEntry();
    eIntakeState =sensorTab.add("Intake State", m_IntakeSubsystem.getIntakeState()).getEntry();
  }

  public void outputToSmartDashboard() {
    SmartDashboard.putBoolean("LowSensor",  m_SequencerSubsystem.getLowSensor());
    SmartDashboard.putNumber("PowerCellCount",  (double)m_SequencerSubsystem.getPowerCellCount());
    SmartDashboard.putString("Intake State",  m_IntakeSubsystem.getIntakeState());
    eLowSensor.setBoolean(m_SequencerSubsystem.getLowSensor());
    ePowerCellCount.forceSetNumber(m_SequencerSubsystem.getPowerCellCount());
    eIntakeState.setString(m_IntakeSubsystem.getIntakeState());
    }
}
