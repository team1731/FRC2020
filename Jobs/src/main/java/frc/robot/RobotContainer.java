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
//import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;

import frc.robot.commands.*;
import frc.robot.autonomous._2_BwdPickup2BallsAndShoot;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LedStringSubsystem;
import frc.robot.subsystems.SequencerSubsystem;
import frc.robot.subsystems.ShootClimbSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;


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
  private final LedStringSubsystem m_LedStringSubsystem = new LedStringSubsystem();


  //private final ExampleCommand m_autoCommand = new ExampleCommand(m_IntakeSubsystem);


  XboxController driverController = new XboxController(0); 
  XboxController operatorController = new XboxController(1); 
  private NetworkTableEntry eLowSensor;
  private NetworkTableEntry ePowerCellCount;
  private NetworkTableEntry eIntakeState;

  // Controller Triggers
  public enum HansTriggers {DR_TRIG_LEFT, DR_TRIG_RIGHT, OP_TRIG_LEFT, OP_TRIG_RIGHT}
  
  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  // initial SubSystems to at rest states
  public void initSubsystems() {
    m_IntakeSubsystem.retract();
    m_SequencerSubsystem.stop();
    m_ShootClimbSubsystem.disable();
    m_LedStringSubsystem.init();
  }
  
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
    //new JoystickButton(operatorController, 5).whenHeld(
    //  new IntakeSeqCommand(m_IntakeSubsystem, m_SequencerSubsystem)
    //);
    // Activate Intake via Operator Left Axis/Trigger
    new HansTrigger(HansTriggers.OP_TRIG_LEFT).whileActiveOnce(
      new IntakeSeqCommand(m_IntakeSubsystem, m_SequencerSubsystem)
    );

    // Shooter
    //new JoystickButton(operatorController, 6).whenHeld(
    //  new ShootSeqCommand(m_ShootClimbSubsystem, m_SequencerSubsystem)
    //);
    // Activate Shooter via Operator Right Axis/Trigger
    new HansTrigger(HansTriggers.OP_TRIG_RIGHT).whileActiveOnce(
      new ShootSeqCommand(m_ShootClimbSubsystem, m_SequencerSubsystem)
    );

    // Climbing Mode
    new StickTrigger().whileActiveContinuous(
      new ClimbingCommand(m_ShootClimbSubsystem, () -> operatorController.getRawAxis(1))
    );
        //.whenReleased(new InstantCommand(m_ShootClimbSubsystem::off, m_ShootClimbSubsystem));

    // Select Shoot or Climb Mode
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
  
  public Command getAutonomousCommand(int autoNum) {
    // An ExampleCommand will run in autonomous
    return new _2_BwdPickup2BallsAndShoot().getCommand(m_IntakeSubsystem, m_SequencerSubsystem, m_ShootClimbSubsystem);
  }
  
  public class StickTrigger extends Trigger {
    public boolean get() {
      //double v = operatorController.getY(Hand.kRight);
      //v = operatorController.getX(Hand.kRight);
      //x = operatorController.getRawAxis(0);
      double y = operatorController.getRawAxis(1);
      return Math.abs(y) < 0.1 ? false : true; 
    }
  }

  // Enables Use of controller axis/trigger by creating a Custom Trigger
  public class HansTrigger extends Trigger {
    HansTriggers desired;
    double triggerValue = 0;

    public HansTrigger(HansTriggers selected) {
      this.desired = selected;
    }

    @Override
    public boolean get() {
      switch (desired) {
        case DR_TRIG_LEFT:
          triggerValue = driverController.getTriggerAxis(Hand.kLeft);
          break;
        case DR_TRIG_RIGHT:
          triggerValue = driverController.getTriggerAxis(Hand.kRight);
          break;
        case OP_TRIG_LEFT:
          triggerValue = operatorController.getTriggerAxis(Hand.kLeft);
          break;
        case OP_TRIG_RIGHT:
          triggerValue = operatorController.getTriggerAxis(Hand.kRight);
          break;
      }
      return (Math.abs(triggerValue) > 0.5);
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
