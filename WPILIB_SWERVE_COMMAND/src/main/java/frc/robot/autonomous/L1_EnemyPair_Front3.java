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
                {{0.0, 0.0, 0.0},    //initial pose
                  {-1.06, -0.2},     // waypoint(s)
                  {-1.9, -0.2},
                  {-2.2, -0.2, 0.0}} // final pose
                ),
        
                new IntakeSeqCommand(m_intake, m_sequence).withTimeout(4)
            ),

            new ParallelCommandGroup(
                new InstantCommand(m_shootclimb::enableShooting, m_shootclimb).withTimeout(4),
                createSwerveCommand(m_robotDrive, "STRAFE TO SHOOT LOCATION 1", TrajectoryDirection.FWD, 
                                    TrajectoryHeading.UNROTATE, 0, new double[][]
                    {{-2.2, -0.2, 0},    //initial pose
                     //{-1, -0.1},     // waypoint(s)
                     //{-2, -2.43},
                    {-0.54, -4.74, 50}} // final pose
                )),

            new WaitCommand(1),
            new ShootSeqCommandAuto(m_shootclimb, m_sequence).withTimeout(1),
            new WaitCommand(getSecondaryDelaySeconds()),

            new ParallelCommandGroup(
                createSwerveCommand(m_robotDrive, "BACKWARD TO FRONT 3", TrajectoryDirection.REV,
                                    TrajectoryHeading.UNROTATE, 70, new double[][]
                {{-0.54, -4.74, -12},    //initial pose
                  {-0.93, -3.45},     // waypoint(s)
                  {-1.81, -3.3},
                  {-1.96, -3.55},
                  {-2.14, -3.4, 65}} // final pose
                ),
        
                new IntakeSeqCommand(m_intake, m_sequence).withTimeout(4)
            ),

            new ParallelCommandGroup(    
                createSwerveCommand(m_robotDrive, "STRAFE TO PICKUP FRONT 3", TrajectoryDirection.REV,
                                    TrajectoryHeading.MAINTAIN, 65, new double[][]
                {{-2.14, -3.4, 65},    //initial pose
                  {-2.11, -3.53},     // waypoint(s)
                  {-1.79, -4.08, 70}} // final pose
                ),
                new IntakeSeqCommand(m_intake, m_sequence).withTimeout(4)
            ),

            new ParallelCommandGroup(
                new InstantCommand(m_shootclimb::enableShooting, m_shootclimb).withTimeout(4),
                createSwerveCommand(m_robotDrive, "STRAFE TO SHOOT LOCATION 2", TrajectoryDirection.FWD, 
                                    TrajectoryHeading.UNROTATE, 0, new double[][]
                    {{-2.14, -3.4, 65},    //initial pose
                     {-0.89, -4.24},     // waypoint(s)
                    {-0.54, -4.74, 50}} // final pose
                )),

            new WaitCommand(1),
            new ShootSeqCommandAuto(m_shootclimb, m_sequence).withTimeout(1)
        );

        // Run path following command, then stop at the end.
        command = commandGroup.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
    }
}