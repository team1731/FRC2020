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

public class InstrumentedSwerveControllerCommand extends SwerveControllerCommand {
    private final DebugOutput debugOutput = new DebugOutput();
    private final Trajectory trajectory;
    private final Supplier<Pose2d> pose;
    private final SwerveDriveKinematics kinematics;
    private final PIDController xController;
    private final PIDController yController;
    private final ProfiledPIDController thetaController;
    private final Consumer<SwerveModuleState[]> outputModuleStates;
    private final Subsystem[] requirements;
    private final ReflectingCSVWriter<DebugOutput> mCSVWriter;
    private final Pose2d finalPose;

    public InstrumentedSwerveControllerCommand(Trajectory trajectory, Supplier<Pose2d> pose,
            SwerveDriveKinematics kinematics, PIDController xController, PIDController yController,
            ProfiledPIDController thetaController, Consumer<SwerveModuleState[]> outputModuleStates,
            Subsystem[] requirements, ReflectingCSVWriter<DebugOutput> mCSVWriter) {
        super(trajectory, pose, kinematics, xController, yController, thetaController, outputModuleStates, requirements);
        this.trajectory = trajectory;
        this.pose = pose;
        this.kinematics = kinematics;
        this.xController = xController;
        this.yController = yController;
        this.thetaController = thetaController;
        this.outputModuleStates = outputModuleStates;
        this.requirements = requirements;
        this.mCSVWriter = mCSVWriter;
        this.finalPose = trajectory.sample(trajectory.getTotalTimeSeconds()).poseMeters;
    }

    @Override
    public void execute(){
        double curTime = Timer.getFPGATimestamp();

        var desiredState = trajectory.sample(curTime);
        var desiredPose = desiredState.poseMeters;
    
        var poseError = desiredPose.relativeTo(pose.get());
    
        double targetXVel = xController.calculate(
            pose.get().getTranslation().getX(),
            desiredPose.getTranslation().getX());
    
        double targetYVel = yController.calculate(
            pose.get().getTranslation().getY(),
            desiredPose.getTranslation().getY());
    
        // The robot will go to the desired rotation of the final pose in the trajectory,
        // not following the poses at individual states.
        double targetAngularVel = thetaController.calculate(
            pose.get().getRotation().getRadians(),
            finalPose.getRotation().getRadians());
    
        double vRef = desiredState.velocityMetersPerSecond;
    
        targetXVel += vRef * poseError.getRotation().getCos();
        targetYVel += vRef * poseError.getRotation().getSin();
    
        var targetChassisSpeeds = new ChassisSpeeds(targetXVel, targetYVel, targetAngularVel);
    
        var targetModuleStates = kinematics.toSwerveModuleStates(targetChassisSpeeds);
    
        outputModuleStates.accept(targetModuleStates);
    
        debugOutput.update(Timer.getFPGATimestamp(), poseError, targetChassisSpeeds, targetModuleStates);
        mCSVWriter.add(debugOutput);
    }
}