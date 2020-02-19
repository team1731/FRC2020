/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
/* Intake Motor (I), Sequence Motor (S)
  Hi Mid Lo | I S
   0  0  0  | 1 0
   0  0  1  | 1 1
   0  1  0  | 1 1
   0  1  1  | 0 1
   1  X  X  | 0 0

   If you switch case 2 & 3, then spacing will be ball width, now it's sensor width
*/
package frc.robot.commands;

import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SequencerSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * An example command that uses an example subsystem.
 */
public class IntakeSeqCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final IntakeSubsystem m_IntakeSubsystem;
  private final SequencerSubsystem m_SeqSubsystem;

  /**
   * Creates a new Intake Sequence Command.
   *
   * @param intakeSubsystem The intake subsystem this command will run on
   * @param seqSubsystem The sequencer subsystem this command will run on
   */
  public IntakeSeqCommand(IntakeSubsystem intakeSubsystem, SequencerSubsystem seqSubsystem) {
    m_IntakeSubsystem = intakeSubsystem;
    m_SeqSubsystem = seqSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakeSubsystem, seqSubsystem);
  }

  // Called when the command is initially scheduled.
  // If it is used as Default command then it gets call all the time
  @Override
  public void initialize() {
    m_IntakeSubsystem.extend();
    //m_SeqSubsystem.stop();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // get necessary input
    /*
    if (m_SeqSubsystem.highSensorHasBall()) {
      m_SeqSubsystem.stop();
      m_IntakeSubsystem.inactive();
    } else {
      if (!m_SeqSubsystem.lowSensorHasBall() && !m_SeqSubsystem.midSensorHasBall()) {
        m_SeqSubsystem.stop();
        m_IntakeSubsystem.active();
      } else if (m_SeqSubsystem.lowSensorHasBall() && m_SeqSubsystem.midSensorHasBall()) {
        m_SeqSubsystem.forward(false);
        m_IntakeSubsystem.inactive();
      } else {
        m_SeqSubsystem.forward(false);
        m_IntakeSubsystem.active();
      }
    }
    */
    
    // if low and high not tripped do something
    // if they are both tripped we do NOTHING
    if(!m_SeqSubsystem.lowSensorHasBall() && !m_SeqSubsystem.highSensorHasBall() ){
      //do something
      if (m_SeqSubsystem.lowSensorHasBall() ||  m_SeqSubsystem.midSensorHasBall()) {
        m_IntakeSubsystem.inactive();
      } else {
        m_IntakeSubsystem.active();
      }
      System.out.println("intake extended");
    }
    else if (m_SeqSubsystem.lowSensorHasBall() && m_SeqSubsystem.highSensorHasBall()){
      m_IntakeSubsystem.inactive();
      m_IntakeSubsystem.retract();
    }
    if((m_SeqSubsystem.lowSensorHasBall() || m_SeqSubsystem.midSensorHasBall()) && !m_SeqSubsystem.highSensorHasBall()){
      m_SeqSubsystem.forward(false); // false indicates forward intaking speed (i.e., NOT shooting)
    }
    else{
      m_SeqSubsystem.stop();
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("IntakeSequenceCommand end interrupted=" + (interrupted?"true":"false"));
    m_SeqSubsystem.stop();
    m_IntakeSubsystem.inactive();
    m_IntakeSubsystem.retract();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_SeqSubsystem.highSensorHasBall(); //m_SeqSubsystem.getMaxPowerCells();
  }
}
