/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.autonomous.F1_Move_Forward;
import frc.robot.autonomous.L1_Placeholder;
import frc.robot.autonomous.M1_Placeholder;
import frc.robot.autonomous.M3_Placeholder;
import frc.robot.autonomous.R1_Placeholder;
import frc.robot.autonomous.T1_Move_Forward;
import frc.robot.autonomous.T2_BwdPickup2Balls;
import frc.robot.autonomous.T3_BwdPickup2BallsAndShoot;
import frc.robot.autonomous.T4_AimAndShoot;
import frc.robot.autonomous._NamedAutoMode;
import frc.robot.autonomous._NotImplementedProperlyException;
import frc.robot.commands.*;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.JevoisVisionSubsystem;
import frc.robot.subsystems.SequencerSubsystem;
import frc.robot.subsystems.TargetingSubsystem;
import frc.robot.subsystems.ShootClimbSubsystem;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  Map<String, _NamedAutoMode> nameAutoModeMap;

  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  //XboxController m_operatorController = new XboxController(OIConstants.kOperatorControllerPort);
  Joystick m_operatorController = new Joystick(OIConstants.kOperatorControllerPort);

  private DriveSubsystem m_robotDrive;
  private IntakeSubsystem m_intake;
  private ShootClimbSubsystem m_shootclimb;
  private SequencerSubsystem m_sequencer;
  private TargetingSubsystem m_targeting;
  private JevoisVisionSubsystem m_vision;

  // Controller Triggers
  public enum HanTriggers {
    DR_TRIG_LEFT, DR_TRIG_RIGHT, OP_TRIG_LEFT, OP_TRIG_RIGHT
  }
  public enum HanMode { MODE_SHOOT, MODE_CLIMB }

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   * 
   * @throws _NotImplementedProperlyException
   */
  public RobotContainer(DriveSubsystem m_robotDrive, IntakeSubsystem m_intake, SequencerSubsystem m_sequencer,
      ShootClimbSubsystem m_shootclimb, TargetingSubsystem m_targeting, JevoisVisionSubsystem m_vision)
      throws _NotImplementedProperlyException {
    this.m_robotDrive = m_robotDrive;
    this.m_intake = m_intake;
    this.m_sequencer = m_sequencer;
    this.m_targeting = m_targeting;
    this.m_vision = m_vision;
    this.m_shootclimb = m_shootclimb;

    nameAutoModeMap = createNamedAutoModeMap();

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
            -m_driverController.getY(Hand.kLeft) * DriveConstants.kMaxSpeedMetersPerSecond,

            // Get the y speed or sideways/strafe speed. We are inverting this because
            // we want a positive value when we pull to the left. Xbox controllers
            // return positive values when you pull to the right by default.
            -m_driverController.getX(Hand.kLeft) * DriveConstants.kMaxSpeedMetersPerSecond,

            // Get the rate of angular rotation. We are inverting this because we want a
            // positive value when we pull to the left (remember, CCW is positive in
            // mathematics). Xbox controllers return positive values when you pull to
            // the right by default.
            -m_driverController.getX(Hand.kRight), true),

            m_robotDrive));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * calling passing it to a {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    // Turn to 90 degrees when the 'X' button is pressed, with a 5 second timeout
    // new JoystickButton(m_driverController, Button.kX.value)
    // .whenPressed(new TurnToAngle(30, m_robotDrive).withTimeout(5));

    // Turn to -90 degrees with a profile when the 'A' button is pressed, with a 5
    // second timeout
    // new JoystickButton(m_driverController, Button.kA.value)
    // .whenPressed(new TurnToAngleProfiled(30, m_robotDrive).withTimeout(5));

    // Activate Intake via Operator Left Axis/Trigger
    new HanTrigger(HanTriggers.DR_TRIG_LEFT).whileActiveContinuous(new IntakeSeqCommand(m_intake, m_sequencer));
    // Activate Intake via Operator Left Front Top - Up is Intaking, Down is Reset 
    //new JoystickButton(m_operatorController, 3).whileActiveContinuous(new IntakeSeqCommand(m_intake, m_sequencer));
    new JoystickButton(m_driverController, 2)
    .whileActiveContinuous(new SeqResetCommand(m_sequencer), true);

    new JoystickButton(m_operatorController, 8)
      //.whenActive(new InstantCommand(m_shootclimb::enableShooting, m_shootclimb))
      .whileActiveContinuous(new JoystickShooter(m_shootclimb, () -> m_operatorController.getRawAxis(4)), false
    );

    // Climbing Command - CURRENT
    new JoystickButton(m_operatorController, 9)
     .whileActiveContinuous(
       new ClimbingCommand(m_shootclimb, () -> m_operatorController.getRawAxis(1)), true
     );


    // Climber Extend
    //new JoystickButton(m_operatorController, 1)
    // .whileActiveOnce(new InstantCommand(m_shootclimb::climbExtend, m_shootclimb));
    // Climber Retract
    //new JoystickButton(m_operatorController, 2)
    // .whileActiveOnce(new InstantCommand(m_shootclimb::climbRetract, m_shootclimb));

    // Activate Shooter via Operator Right Axis/Trigger
    //new HanTrigger(HanTriggers.DR_TRIG_RIGHT).whileActiveContinuous(new ShootSeqCommand(m_shootclimb, m_sequencer));
    // Shooting
    new JoystickButton(m_operatorController, 12).whileActiveContinuous(
      new ShootSeqCommand(m_shootclimb, m_sequencer), true //<---NOTE: we think this got called at least once when we ran autonomous
    );
    //new ModeTrigger(HanMode.MODE_SHOOT).whenActive(
    //  new InstantCommand(m_shootclimb::enableShooting, m_shootclimb)
    //);

    // Sequencer ejects works will button is held
    new JoystickButton(m_driverController, 7)
    .whenHeld(new InstantCommand(m_sequencer::reverse, m_sequencer))
    .whenReleased(new InstantCommand(m_sequencer::stop, m_sequencer));

    // Climbing Command
    //new ModeTrigger(HanMode.MODE_CLIMB).whileActiveContinuous(
    //  new ClimbingCommand(m_shootclimb, () -> m_operatorController.getRawAxis(1)), true
    //);

    /*
    // Intake & Sequencer ejects works will button is held
    new JoystickButton(m_operatorController, 1)
        .whenHeld(new ParallelCommandGroup(new InstantCommand(m_intake::eject, m_intake),
            new InstantCommand(m_sequencer::reverse, m_sequencer)))
        .whenReleased(new ParallelCommandGroup(new InstantCommand(m_intake::retract, m_intake),
            new InstantCommand(m_sequencer::stop, m_sequencer)));

    // Activate Intake via Operator Left Axis/Trigger
    //new HanTrigger(HanTriggers.OP_TRIG_LEFT).whileActiveContinuous(new IntakeSeqCommand(m_intake, m_sequencer));

    // Activate Shooter via Operator Right Axis/Trigger
    //new HanTrigger(HanTriggers.OP_TRIG_RIGHT).whileActiveOnce(new ShootSeqCommand(m_shootclimb, m_sequencer));

    // Select Shoot or Climb Mode
    new JoystickButton(m_operatorController, 3).whenPressed(new InstantCommand(m_shootclimb::modeClimb, m_shootclimb));
    // .whenReleased(new InstantCommand(m_ShootClimbSubsystem::off,
    // m_ShootClimbSubsystem));
    new JoystickButton(m_operatorController, 4)
        // .whenPressed(new InstantCommand(m_ShootClimbSubsystem::on,
        // m_ShootClimbSubsystem))
        .whenReleased(new InstantCommand(m_shootclimb::modeShoot, m_shootclimb));

    new JoystickButton(m_operatorController, 8)
    // .whenPressed(new InstantCommand(m_ShootClimbSubsystem::on,
    // m_ShootClimbSubsystem))

    
      .toggleWhenPressed(new ParallelCommandGroup[] {
        new InstantCommand(m_shootclimb::modeShoot, m_shootclimb)

    });
    */
  }

  public _NamedAutoMode getNamedAutonomousCommand(String autoSelected) throws _NotImplementedProperlyException {
    String autoMode = "";
    int initialDelaySeconds = 0;
    int secondaryDelaySeconds = 0;
    if(autoSelected.length() > 1){
      autoMode = autoSelected.substring(0, 2);
    }
    if(autoSelected.length() > 2){
      try{
        initialDelaySeconds = Integer.parseInt(autoSelected.substring(2, 2));
      }
      catch(Exception e){
        System.out.println("INITIAL DELAY did not parse -- defaulting to 0 seconds!!!");
      }
    }
    if(autoSelected.length() > 3){
      try{
        secondaryDelaySeconds = Integer.parseInt(autoSelected.substring(3, 3));
      }
      catch(Exception e){
        System.out.println("SECONDARY DELAY did not parse -- defaulting to 0 seconds!!!");
      }
    }

    _NamedAutoMode selectedAutoMode = nameAutoModeMap.get(autoMode);
    if(selectedAutoMode == null){
      System.err.println("SELECTED MODE NOT IMPLEMENTED -- DEFAULT TO F1_MOVE_FORWARD!!!");
      selectedAutoMode = new _NamedAutoMode(new F1_Move_Forward(m_robotDrive));
    }
    selectedAutoMode.delayableStrafingAutoMode.setInitialDelaySeconds(initialDelaySeconds);
    selectedAutoMode.delayableStrafingAutoMode.setSecondaryDelaySeconds(secondaryDelaySeconds);

    return selectedAutoMode;
  }

  private Map<String, _NamedAutoMode> createNamedAutoModeMap() throws _NotImplementedProperlyException {
    _NamedAutoMode mode;
    Map<String, _NamedAutoMode> myMap = new HashMap<String, _NamedAutoMode>();

      // DEFAULT AUTO -- MOVE FORWARD
      //
      mode = new _NamedAutoMode(new F1_Move_Forward(m_robotDrive));
      myMap.put(mode.code, mode);

      // TEST AUTO ROUTINES
      //
      mode = new _NamedAutoMode(new T1_Move_Forward(m_robotDrive));
      myMap.put(mode.code, mode);

      mode = new _NamedAutoMode(new T2_BwdPickup2Balls(m_robotDrive));
      myMap.put(mode.code, mode);

      mode = new _NamedAutoMode(new T3_BwdPickup2BallsAndShoot(m_robotDrive, m_intake, m_sequencer, m_shootclimb, m_vision, m_targeting));
      myMap.put(mode.code, mode);
                    
      mode = new _NamedAutoMode(new T4_AimAndShoot(m_robotDrive, m_sequencer, m_shootclimb, m_vision, m_targeting));
      myMap.put(mode.code, mode);

      // FOR HAYMARKET: R1, L1, M1, M3
      //
      mode = new _NamedAutoMode(new R1_Placeholder(m_robotDrive, m_intake, m_sequencer, m_shootclimb, m_vision, m_targeting));
      myMap.put(mode.code, mode);

      mode = new _NamedAutoMode(new L1_Placeholder(m_robotDrive, m_intake, m_sequencer, m_shootclimb, m_vision, m_targeting));
      myMap.put(mode.code, mode);

      mode = new _NamedAutoMode(new M1_Placeholder(m_robotDrive, m_intake, m_sequencer, m_shootclimb, m_vision, m_targeting));
      myMap.put(mode.code, mode);
                    
      mode = new _NamedAutoMode(new M3_Placeholder(m_robotDrive, m_intake, m_sequencer, m_shootclimb, m_vision, m_targeting));
      myMap.put(mode.code, mode);

      return myMap;
  }
    
  public class StickTrigger extends Trigger {
    public boolean get() {
      //double v = operatorController.getY(Hand.kRight);
      //v = operatorController.getX(Hand.kRight);
      //x = operatorController.getRawAxis(0);
      double y = m_operatorController.getRawAxis(1);
      return Math.abs(y) < 0.2 ? false : true; 
    }
  }

  // Enables Use of controller axis/trigger by creating a Custom Trigger
  public class HanTrigger extends Trigger {
    HanTriggers desired;
    double triggerValue = 0;

    public HanTrigger(HanTriggers selected) {
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
          //triggerValue = m_operatorController.getTriggerAxis(Hand.kLeft);
          break;
        case OP_TRIG_RIGHT:
          //triggerValue = m_operatorController.getTriggerAxis(Hand.kRight);
          break;
      }
      return (Math.abs(triggerValue) > 0.5);
    }
  }

  public class ModeTrigger extends Trigger {
    HanMode mode;
    boolean result;
    public ModeTrigger (HanMode mode) {
      this.mode = mode;
    }

    public boolean get() {
      boolean left = m_operatorController.getRawButton(1);
      boolean right = m_operatorController.getRawButton(12);
      switch (mode) {
        case MODE_SHOOT:
          result = ((!left) && (!right));
          break;
        case MODE_CLIMB:
          result = (left && right);
          break;
      }
      //double v = operatorController.getY(Hand.kRight);
      return result; 
    }
  }

}
