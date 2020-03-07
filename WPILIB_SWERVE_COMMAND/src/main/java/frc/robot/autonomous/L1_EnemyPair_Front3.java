package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import frc.robot.commands.IntakeSeqCommand;
import frc.robot.commands.ShootSeqCommandAuto;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.JevoisVisionSubsystem;
import frc.robot.subsystems.SequencerSubsystem;
import frc.robot.subsystems.ShootClimbSubsystem;

public class L1_EnemyPair_Front3 extends _DelayableStrafingAutoMode {
    public L1_EnemyPair_Front3(DriveSubsystem m_robotDrive, IntakeSubsystem m_intake, SequencerSubsystem m_sequence,
            ShootClimbSubsystem m_shootclimb, JevoisVisionSubsystem m_vision) {
                
        SequentialCommandGroup commandGroup = new SequentialCommandGroup(
            new WaitCommand(getInitialDelaySeconds()),

            // ENEMY PAIR
            new ParallelCommandGroup(
                createSwerveCommand(m_robotDrive, "BACKWARD TO ENEMY PAIR", TrajectoryDirection.REV, 
                                    TrajectoryHeading.UNROTATE, 15, new double[][]
                {{0.0, 0.0, 0.0},    // initial pose
                 {-2.41, -0.15, 0.0}}      // final pose
                ),
        
                new IntakeSeqCommand(m_intake, m_sequence, true).withTimeout(4)
            ),

            new ParallelCommandGroup(
                new InstantCommand(m_shootclimb::enableShooting, m_shootclimb).withTimeout(4),
                createSwerveCommand(m_robotDrive, "STRAFE TO SHOOT LOCATION 1", TrajectoryDirection.FWD, 
                                    TrajectoryHeading.UNROTATE, 0, new double[][]
                {{-2.41, -0.15, 0.0},    // initial pose
                    {-0.49, -1.84},       // waypoint(s)
                    {-0.25, -4.95, -6.7}} // final pose
                )
            ),

            new WaitCommand(1),
            new ShootSeqCommandAuto(m_shootclimb, m_sequence).withTimeout(1),
            new WaitCommand(getSecondaryDelaySeconds()),

            // FRONT 3
            new ParallelCommandGroup(
                createSwerveCommand(m_robotDrive, "BACKWARD TO FRONT 3", TrajectoryDirection.REV,
                                    TrajectoryHeading.MAINTAIN, 34.4, new double[][]
                {{-0.25, -4.95, -6.7}, // initial pose
                 {-1.97, -3.11, 34.4},      // waypoint(s)
                 {-1.91, -3.96, 39}}   // final pose
                ),
        
                new IntakeSeqCommand(m_intake, m_sequence, true).withTimeout(4)
            ),

            new ParallelCommandGroup(
                new InstantCommand(m_shootclimb::enableShooting, m_shootclimb).withTimeout(4),
                createSwerveCommand(m_robotDrive, "STRAFE TO SHOOT LOCATION 2", TrajectoryDirection.FWD, 
                                    TrajectoryHeading.UNROTATE, 0, new double[][]
                {{-1.91, -3.96, 39},   // initial pose
                 {-0.75, -4.01},      // waypoint(s)
                 {-0.25, -4.95, -6.7}}  // final pose
                )
            ),

            new WaitCommand(1),
            new ShootSeqCommandAuto(m_shootclimb, m_sequence).withTimeout(1)
        );

        // Run path following command, then stop at the end.
        command = commandGroup.andThen(() -> m_robotDrive.drive(0, 0, 0, false)).andThen(() -> m_shootclimb.stopShooting());
    }
}