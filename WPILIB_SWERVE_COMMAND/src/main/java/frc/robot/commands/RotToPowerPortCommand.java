package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.XboxConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.JevoisVisionSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.vision.ShooterAimingParameters;

public class RotToPowerPortCommand extends CommandBase {

    private final LimeLightSubsystem m_vision;
    private final DriveSubsystem m_drive;
    private final XboxController m_driverController;

    public RotToPowerPortCommand(LimeLightSubsystem visionSubsystem, DriveSubsystem driveSubsystem, XboxController driveController){
        m_vision = visionSubsystem;
        m_drive = driveSubsystem;
        m_driverController = driveController;
    }

    @Override
    public void initialize(){
        m_drive.setStickControlledHeading(false);
        m_vision.enableLED();
    }

    @Override
    public void execute(){
        if(m_vision.hasTarget()){
            SmartDashboard.putBoolean("Vis_HasTarget", true);
            
            double targetAngle = m_drive.getHeading() + m_vision.getLastTarget().getX();
            SmartDashboard.putNumber("Vis_TargetAngle", targetAngle);
            m_drive.setHeadingControllerGoal(targetAngle);
        } else {
            SmartDashboard.putBoolean("Vis_HasTarget", false);
        }
    }

    @Override
    public void end(boolean interrupted){
        m_drive.setStickControlledHeading(true);
        m_vision.disableLED();
        SmartDashboard.putBoolean("Vis_HasTarget", false);
    }

    @Override
    public boolean isFinished(){
        //When the right bumper is released, stop the command
        return !m_driverController.getRawButton(XboxConstants.kRBumper);
    } 

}