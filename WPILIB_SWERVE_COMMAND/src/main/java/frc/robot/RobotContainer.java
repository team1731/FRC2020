/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.autonomous._0_MoveForward;
import frc.robot.autonomous._1_BwdPickup2Balls;
import frc.robot.autonomous._2_BwdPickup2BallsAndShoot;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TargetingSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  Command[] autoCommands;
  DriveSubsystem m_robotDrive;

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

  private IntakeSubsystem m_intake;
  private ShooterSubsystem m_shooter;
  private TargetingSubsystem m_targeting;
  private VisionSubsystem m_vision;
  private ClimberSubsystem m_climber;

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer(DriveSubsystem m_robotDrive, IntakeSubsystem m_intake, ShooterSubsystem m_shooter, TargetingSubsystem m_targeting, VisionSubsystem m_vision, ClimberSubsystem m_climber) {
    this.m_robotDrive = m_robotDrive;
    this.m_intake = m_intake;
    this.m_shooter = m_shooter;
    this.m_targeting = m_targeting;
    this.m_vision = m_vision;
    this.m_climber = m_climber;
    
    autoCommands = setupAutoCommands();
    
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
    new JoystickButton(m_driverController, Button.kX.value)
        .whenPressed(new TurnToAngle(30, m_robotDrive).withTimeout(5));

    // Turn to -90 degrees with a profile when the 'A' button is pressed, with a 5 second timeout
    new JoystickButton(m_driverController, Button.kA.value)
        .whenPressed(new TurnToAngleProfiled(30, m_robotDrive).withTimeout(5));

  }

  public Command getAutonomousCommand(int autoNum){
    Command autonomousCommand = null;
    if(autoNum >= 0 && autoNum < autoCommands.length){
      autonomousCommand = autoCommands[autoNum];
    }
    return autonomousCommand;
  }

  private Command[] setupAutoCommands(){
    Command[] autoCommands = new Command[]{
      new _0_MoveForward().getCommand(m_robotDrive),
      new _1_BwdPickup2Balls().getCommand(m_robotDrive),                                                               
      new _2_BwdPickup2BallsAndShoot().getCommand(m_robotDrive, m_intake, m_shooter, m_vision, m_targeting)
    };
    return autoCommands;
  }
}
