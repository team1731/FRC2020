package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.OpConstants;
import edu.wpi.first.wpilibj.PWMTalonFX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import frc.robot.util.DebugOutput;
import frc.robot.util.ReflectingCSVWriter;

public class IntakeSubsystem extends SubsystemBase {

  private final PWMTalonFX mTalonIntake;
  private final DoubleSolenoid mIntakeSolenoid;
  private String mTalonState;
  
  /**
   * Creates a new IntakeSubsystem.
   */
	public IntakeSubsystem() {
    mTalonIntake = new PWMTalonFX(OpConstants.kMotorPWMIntake);
    mIntakeSolenoid = Constants.makeDoubleSolenoidForIds(1, OpConstants.kIntakeRetract, OpConstants.kIntakeExtend);
    mTalonState = "Off";
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * Enables the intake by extending solenoid & turning on motor.
   */
  public void extend() {
    mIntakeSolenoid.set(DoubleSolenoid.Value.kForward);
    mTalonIntake.setSpeed(OpConstants.kMotorIntakeFwdSpeed);
    mTalonState = "Extending/Fwd";
  }

  /**
   * Enables the intake by extending solenoid & turning on motor.
   */
  public void eject() {
    //mIntakeSolenoid.set(true);
    mTalonIntake.setSpeed(OpConstants.kMotorIntakeRevSpeed);
    mTalonState = "Ejecting/Rev";
  }

  /**
   * Enables the intake by retracting solenoid & turning off motor.
   */
  public void retract() {
    mIntakeSolenoid.set(DoubleSolenoid.Value.kReverse);
    mTalonIntake.setSpeed(0);
    mTalonState = "Retracted/Off";
  }

  public String getIntakeState() {
    return(mTalonState);
  }
}
