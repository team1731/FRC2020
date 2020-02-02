package frc.robot;

import java.util.Arrays;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.control.Controls;
import frc.robot.subsystem.DriveSubsystem;
import java.util.Date;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import org.strykeforce.thirdcoast.swerve.Wheel;
import org.usfirst.frc.team1731.robot.loops.Looper;
import org.usfirst.frc.team1731.robot.subsystems.Superstructure;
import org.usfirst.frc.team1731.robot.SubsystemManager;
import org.usfirst.frc.team1731.robot.ControlBoardInterface;
import org.usfirst.frc.team1731.robot.GamepadControlBoard;
import org.usfirst.frc.team1731.robot.subsystems.Intake;
import org.usfirst.frc.team1731.robot.subsystems.Shooter;
import org.usfirst.frc.team1731.robot.Constants;

public class Robot extends TimedRobot {
  private final Superstructure mSuperstructure = Superstructure.getInstance();
  private static final Logger logger = LoggerFactory.getLogger(Robot.class);
  // public static final DriveSubsystem DRIVE = new DriveSubsystem();

  // Controls initialize Commands so this should be instantiated last to prevent

  private final SubsystemManager mSubsystemManager = new SubsystemManager(
      Arrays.asList(Superstructure.getInstance(), Intake.getInstance(), Shooter.getInstance()));
  // NullPointerExceptions in commands that require() Subsystems above.
  private final ControlBoardInterface mControlBoard = GamepadControlBoard.getInstance();
  public static final Controls CONTROLS = new Controls();

  private CANSparkMax m_motor;
  private CANPIDController m_pidController;
  private CANEncoder m_encoder;
  private static final int deviceID = 1;
  private Wheel[] wheelObjects;
  private boolean mLoopersAreRunning = false;

  private Looper mEnabledLooper = new Looper();

  @Override
  public void robotInit() {
    // logger.info("<b>Robot</b>: robotInit Started");
    // autoCode = SmartDashboard.getString("AutoCode", Constants.kDefaultAutoMode);
    // // or R
    System.out.println("Today is " + new Date().toString());
    // DRIVE.zeroAzimuthEncoders();
    // DRIVE.zeroGyro();

    // logger.info("<b>Robot</b>: robotInit Finished");
    // just some example lines i used to prove that code completion still works!
    // m_motor = new CANSparkMax(deviceID, MotorType.kBrushless);
    // m_motor.disable();
    mSubsystemManager.registerEnabledLoops(mEnabledLooper);
  }

  /**
   * Initializes the robot for the beginning of teleop
   */
  @Override
  public void teleopInit() {
    try {
      // IF TELEOP DOESN"T WORK PUT THESE LINES BACK IN that are shifted to right and
      // commented out below!!!!!!!!!!!!!!!!!!!!!!!!!!!
      // Start loopers
      mSuperstructure.reloadConstants();
      mEnabledLooper.start();
      mLoopersAreRunning = true;

      // mSuperstructure.setOverrideCompressor(false);
    } catch (final Throwable t) {
      // CrashTracker.logThrowableCrash(t);
      // throw t;
    }
  }

  /**
   * This function is called periodically during operator control.
   * 
   * The code uses state machines to ensure that no matter what buttons the driver
   * presses, the robot behaves in a safe and consistent manner.
   * 
   * Based on driver input, the code sets a desired state for each subsystem. Each
   * subsystem will constantly compare its desired and actual states and act to
   * bring the two closer.
   */
  @Override
  public void teleopPeriodic() {
    periodic();
  }

  public void periodic() {
    boolean pickupPowerCell = mControlBoard.getPickupBall();
    boolean ejectPowerCell = mControlBoard.getEjectBall();
    boolean shoot = mControlBoard.getShootBall();
    boolean climbExtend = mControlBoard.getClimberExtend();
    boolean climbRetract = mControlBoard.getClimberRetract();

    if (pickupPowerCell) {
      mSuperstructure.setWantedState(Superstructure.WantedState.POWERCELL_INTAKE);
    } else if (ejectPowerCell) {
      mSuperstructure.setWantedState(Superstructure.WantedState.POWERCELL_EJECT);
    } else if (shoot) {
      mSuperstructure.setWantedState(Superstructure.WantedState.SHOOT);
    } else if (climbExtend) {
      mSuperstructure.setWantedState(Superstructure.WantedState.CLIMBING_EXTEND);
    } else if (climbRetract) {
      mSuperstructure.setWantedState(Superstructure.WantedState.CLIMBING_RETRACT);
    } else {
      mSuperstructure.setWantedState(Superstructure.WantedState.IDLE);
    }
    /*
     * try { //Scheduler.getInstance().run();
     * //logger.info("<b>Robot</b>: teleopPeriodic finished");
     * SmartDashboard.putNumber("Periodic", 1); }
     */
    SmartDashboard.putBoolean("PowerCell Pickup", pickupPowerCell);
    SmartDashboard.putBoolean("PowerCell Eject", ejectPowerCell);
    SmartDashboard.putBoolean("Shoot", shoot);
    // SmartDashboard.putBoolean("Sequencer Low Sensor", sequencerLowSensor.get());
    mSuperstructure.outputToSmartDashboard();
  }

  @Override
  public void disabledPeriodic() {

    if (wheelObjects == null) {
      // wheelObjects = DRIVE.getWheelObjects();
    } else if (wheelObjects.length == 4) {
      for (int i = 0; i < wheelObjects.length; i++) {
        final Wheel wheel = wheelObjects[i];
        if (wheel.m_encoder != null) {
          SmartDashboard.putNumber("Wheel" + wheel.wheelID + " Encoder Pos", wheel.m_encoder.getPosition());
          SmartDashboard.putNumber("Wheel" + wheel.wheelID + " Encoder Get Abs", wheel.getAzimuthAbsolutePosition());
        }
      }
    }
  }

  /**
   * Helper function that is called in all periodic functions
   */
  public void allPeriodic() {
    mSubsystemManager.outputToSmartDashboard();
    mSubsystemManager.writeToLog();
    SmartDashboard.putString("allPeriodic", "Patrick");
  }
}
