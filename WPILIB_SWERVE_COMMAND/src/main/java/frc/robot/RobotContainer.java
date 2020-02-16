/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;
import edu.wpi.first.wpilibj.GenericHID.Hand;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.autonomous._0_MoveForward;
import frc.robot.autonomous._1_BwdPickup2Balls;
import frc.robot.autonomous._2_BwdPickup2BallsAndShoot;
import frc.robot.autonomous._3_AimAndShoot;
import frc.robot.commands.*;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SequencerSubsystem;
import frc.robot.subsystems.TargetingSubsystem;
import frc.robot.subsystems.ShootClimbSubsystem;
import frc.robot.subsystems.VisionSubsystem;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OIConstants;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private NamedAutoCommand[] namedAutoCommands;

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  XboxController m_operatorController = new XboxController(OIConstants.kOperatorControllerPort); 

  private DriveSubsystem m_robotDrive;
  private IntakeSubsystem m_intake;
  private ShootClimbSubsystem m_shootclimb;
  private SequencerSubsystem m_sequencer;
  private TargetingSubsystem m_targeting;
  private VisionSubsystem m_vision;

  // Controller Triggers
  public enum HansTriggers {DR_TRIG_LEFT, DR_TRIG_RIGHT, OP_TRIG_LEFT, OP_TRIG_RIGHT}
  
  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer(DriveSubsystem m_robotDrive, IntakeSubsystem m_intake, SequencerSubsystem m_sequencer, ShootClimbSubsystem m_shootclimb, TargetingSubsystem m_targeting, VisionSubsystem m_vision) {
    this.m_robotDrive = m_robotDrive;
    this.m_intake = m_intake;
    this.m_sequencer = m_sequencer;
    this.m_targeting = m_targeting;
    this.m_vision = m_vision;
    this.m_shootclimb = m_shootclimb;
    
    namedAutoCommands = setupNamedAutoCommands();
    
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    // Set the default drive command to split-stick arcade drive
    m_robotDrive.setDefaultCommand(
        // A split-stick arcade command, with forward/backward controlled by the left
        // hand, and turning controlled by the right.
        new RunCommand(() -> m_robotDrive.drive(
            // Get the x speed. We are inverting this because Xbox controllers return
            // negative values when we push forward.
            -m_driverController.getY(GenericHID.Hand.kLeft),

            // Get the y speed or sideways/strafe speed. We are inverting this because
            // we want a positive value when we pull to the left. Xbox controllers
            // return positive values when you pull to the right by default.
            -m_driverController.getX(GenericHID.Hand.kLeft),

            // Get the rate of angular rotation. We are inverting this because we want a
            // positive value when we pull to the left (remember, CCW is positive in
            // mathematics). Xbox controllers return positive values when you pull to
            // the right by default.
            -m_driverController.getX(GenericHID.Hand.kRight), true),
            
            m_robotDrive)                                                                          
        );
  }

/**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    // Turn to 90 degrees when the 'X' button is pressed, with a 5 second timeout
    //new JoystickButton(m_driverController, Button.kX.value)
    //    .whenPressed(new TurnToAngle(30, m_robotDrive).withTimeout(5));

    // Turn to -90 degrees with a profile when the 'A' button is pressed, with a 5 second timeout
    //new JoystickButton(m_driverController, Button.kA.value)
    //    .whenPressed(new TurnToAngleProfiled(30, m_robotDrive).withTimeout(5));


    // Intake & Sequencer ejects works will button is held
    new JoystickButton(m_operatorController, 1).whenHeld(new ParallelCommandGroup(
      new InstantCommand(m_intake::eject, m_intake),
      new InstantCommand(m_sequencer::reverse, m_sequencer)
      )).whenReleased(new ParallelCommandGroup(
        new InstantCommand(m_intake::retract, m_intake),
        new InstantCommand(m_sequencer::stop, m_sequencer)
      )
    );

    // Activate Intake via Operator Left Axis/Trigger
    new HansTrigger(HansTriggers.OP_TRIG_LEFT).whileActiveOnce(
      new IntakeSeqCommand(m_intake, m_sequencer)
    );

    // Activate Shooter via Operator Right Axis/Trigger
    new HansTrigger(HansTriggers.OP_TRIG_RIGHT).whileActiveOnce(
      new ShootSeqCommand(m_shootclimb, m_sequencer)
    );

    // Climbing Mode
    new StickTrigger().whileActiveContinuous(
      new ClimbingCommand(m_shootclimb, () -> m_operatorController.getRawAxis(1))
    );

    // Select Shoot or Climb Mode
    new JoystickButton(m_operatorController, 3)
        .whenPressed(new InstantCommand(m_shootclimb::modeClimb, m_shootclimb));
        //.whenReleased(new InstantCommand(m_ShootClimbSubsystem::off, m_ShootClimbSubsystem));
    new JoystickButton(m_operatorController, 4)
        //.whenPressed(new InstantCommand(m_ShootClimbSubsystem::on, m_ShootClimbSubsystem))
        .whenReleased(new InstantCommand(m_shootclimb::modeShoot, m_shootclimb));
    
  }

  public NamedAutoCommand getNamedAutonomousCommand(int autoNum){
    if(autoNum < 0 || autoNum >= namedAutoCommands.length){
      System.out.println("specified AUTO NUM (" + autoNum + ") is INVALID -- defaulting to MOVE FORWARD!!!");
      autoNum = 0;
    }
    return namedAutoCommands[autoNum];
  }

  private NamedAutoCommand[] setupNamedAutoCommands(){
    NamedAutoCommand[] namedAutoCommands = new NamedAutoCommand[]{
      new NamedAutoCommand("0 - MOVE FORWARD",
                           new _0_MoveForward().getCommand(m_robotDrive)),
      new NamedAutoCommand("1 = BWD PICKUP 2 BALLS",
                           new _1_BwdPickup2Balls().getCommand(m_robotDrive)),                                                               
      new NamedAutoCommand("2 - BWD PICKUP 2 BALLS AND SHOOT",
                           new _2_BwdPickup2BallsAndShoot().getCommand(m_robotDrive, m_intake, m_sequencer, m_shootclimb, m_vision, m_targeting)),
      new NamedAutoCommand("3 - AIM AND SHOOT",
                           new _3_AimAndShoot().getCommand(m_robotDrive, m_sequencer, m_shootclimb, m_vision, m_targeting))
    };
    return namedAutoCommands;
  }
  
  public class StickTrigger extends Trigger {
    public boolean get() {
      //double v = operatorController.getY(Hand.kRight);
      //v = operatorController.getX(Hand.kRight);
      //x = operatorController.getRawAxis(0);
      double y = m_operatorController.getRawAxis(1);
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
          triggerValue = m_driverController.getTriggerAxis(Hand.kLeft);
          break;
        case DR_TRIG_RIGHT:
          triggerValue = m_driverController.getTriggerAxis(Hand.kRight);
          break;
        case OP_TRIG_LEFT:
          triggerValue = m_operatorController.getTriggerAxis(Hand.kLeft);
          break;
        case OP_TRIG_RIGHT:
          triggerValue = m_operatorController.getTriggerAxis(Hand.kRight);
          break;
      }
      return (Math.abs(triggerValue) > 0.5);
    }
  }

}
