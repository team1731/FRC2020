package frc.robot.subsystems;

import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.util.DebugOutput;
import frc.robot.util.ReflectingCSVWriter;

@Deprecated
public class InstrumentedSwerveControllerCommand  extends SwerveControllerCommand {
    private final DebugOutput m_debugOutput = new DebugOutput();
    private final Trajectory m_trajectory;
    private final Supplier<Pose2d> m_pose;
    private final SwerveDriveKinematics m_kinematics;
    private final PIDController m_xController;
    private final PIDController m_yController;
    private final ProfiledPIDController m_thetaController;
    private final Consumer<SwerveModuleState[]> m_outputModuleStates;
    private final ReflectingCSVWriter<DebugOutput> m_mCSVWriter;
    private Pose2d m_finalPose;
    private final Timer m_timer = new Timer();

    public InstrumentedSwerveControllerCommand(Trajectory trajectory, Supplier<Pose2d> pose,
            SwerveDriveKinematics kinematics, PIDController xController, PIDController yController,
            ProfiledPIDController thetaController, Consumer<SwerveModuleState[]> outputModuleStates,
            DriveSubsystem m_robotDrive, ReflectingCSVWriter<DebugOutput> mCSVWriter) {
        super(trajectory, pose, kinematics, xController, yController, thetaController, outputModuleStates, m_robotDrive);
        m_trajectory = trajectory;
        m_pose = pose;
        m_kinematics = kinematics;
        m_xController = xController;
        m_yController = yController;
        m_thetaController = thetaController;
        m_outputModuleStates = outputModuleStates;
        m_mCSVWriter = mCSVWriter;
    }

  @Override
  public void initialize() {
    // Sample final pose to get robot rotation
    m_finalPose  = m_trajectory.sample(m_trajectory.getTotalTimeSeconds()).poseMeters;

    m_timer.reset();
    m_timer.start();
  }

    @Override
    public void execute(){
        double curTime = Timer.getFPGATimestamp();

        var desiredState = m_trajectory.sample(curTime);
        var desiredPose = desiredState.poseMeters;
    
        var poseError = desiredPose.relativeTo(m_pose.get());
    
        double targetXVel = m_xController.calculate(
            m_pose.get().getTranslation().getX(),
            desiredPose.getTranslation().getX());
    
        double targetYVel = m_yController.calculate(
            m_pose.get().getTranslation().getY(),
            desiredPose.getTranslation().getY());
    
        // The robot will go to the desired rotation of the final pose in the trajectory,
        // not following the poses at individual states.
        double targetAngularVel = m_thetaController.calculate(
            m_pose.get().getRotation().getRadians(),
            m_finalPose.getRotation().getRadians());
    
        double vRef = desiredState.velocityMetersPerSecond;
    
        targetXVel += vRef * poseError.getRotation().getCos();
        targetYVel += vRef * poseError.getRotation().getSin();
    
        var targetChassisSpeeds = new ChassisSpeeds(targetXVel, targetYVel, targetAngularVel);
    
        var targetModuleStates = m_kinematics.toSwerveModuleStates(targetChassisSpeeds);
    
        m_outputModuleStates.accept(targetModuleStates);
    
        m_debugOutput.update(curTime, poseError, targetChassisSpeeds, targetModuleStates);
        m_mCSVWriter.add(m_debugOutput);
    }

    @Override
    public void end(boolean interrupted) {
      m_timer.stop();
    }
  
    @Override
    public boolean isFinished() {
      return m_timer.hasPeriodPassed(m_trajectory.getTotalTimeSeconds());
    }
}