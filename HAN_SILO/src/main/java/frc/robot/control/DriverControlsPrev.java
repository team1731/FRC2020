package frc.robot.control;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.PrintCommand;
import frc.robot.command.ZeroGyroCommand;

/** This assumes the use of an Interlink X Flight Simulator controller. */
@SuppressWarnings("unused")
public class DriverControlsPrev {

  private final Joystick joystick;

  DriverControlsPrev(int port) {
    joystick = new Joystick(port);

    // Shoulder switches
    new JoystickButton(joystick, Shoulder.LEFT_DOWN.id).whenPressed(log(Shoulder.LEFT_DOWN));
    new JoystickButton(joystick, Shoulder.LEFT_UP.id).whenPressed(log(Shoulder.LEFT_UP));
    new JoystickButton(joystick, Shoulder.RIGHT_DOWN.id).whenPressed(log(Shoulder.RIGHT_DOWN));

    // Push-buttons
    new JoystickButton(joystick, Button.RESET.id).whenPressed(new ZeroGyroCommand());

    new JoystickButton(joystick, Button.HAMBURGER.id).whenPressed(log(Button.HAMBURGER));
    new JoystickButton(joystick, Button.X.id).whenPressed(log(Button.X));
    new JoystickButton(joystick, Button.UP.id).whenPressed(log(Button.UP));
    new JoystickButton(joystick, Button.DOWN.id).whenPressed(log(Button.DOWN));

    // Trim Switches
    new JoystickButton(joystick, Trim.LEFT_X_POS.id).whenPressed(log(Trim.LEFT_X_POS));
    new JoystickButton(joystick, Trim.LEFT_X_NEG.id).whenPressed(log(Trim.LEFT_X_NEG));
    new JoystickButton(joystick, Trim.LEFT_Y_POS.id).whenPressed(log(Trim.LEFT_Y_POS));
    new JoystickButton(joystick, Trim.LEFT_Y_NEG.id).whenPressed(log(Trim.LEFT_Y_NEG));
    new JoystickButton(joystick, Trim.RIGHT_X_POS.id).whenPressed(log(Trim.RIGHT_X_POS));
    new JoystickButton(joystick, Trim.RIGHT_X_NEG.id).whenPressed(log(Trim.RIGHT_X_NEG));
    new JoystickButton(joystick, Trim.RIGHT_Y_POS.id).whenPressed(log(Trim.RIGHT_Y_POS));
    new JoystickButton(joystick, Trim.RIGHT_Y_NEG.id).whenPressed(log(Trim.RIGHT_Y_NEG));
  }

  /** Left stick X (up-down) axis. */
  public double getForward() {
    return -joystick.getRawAxis(Axis.LEFT_Y.id);
  }

  /** Left stick Y (left-right) axis. */
  public double getStrafe() {
    return joystick.getRawAxis(Axis.LEFT_X.id);
  }

  /** Right stick Y (left-right) axis. */
  public double getYaw() {
    return joystick.getRawAxis(Axis.RIGHT_X.id);
  }

  /** Tuner knob. */
  public double getTuner() {
    return 0.0; //joystick.getRawAxis(Axis.TUNER.id);
  }

  /** Left slider on back of controller. */
  public double getLeftBackAxis() {
    return 0.0; //joystick.getRawAxis(Axis.LEFT_BACK.id);
  }

  /** Right slider on back of controller. */
  public double getRightBackAxis() {
    return 0.0; //joystick.getRawAxis(Axis.RIGHT_BACK.id);
  }

  private <E extends Enum<E>> Command log(E control) {
    return new PrintCommand(control.toString());
  }

  public enum Axis {
    // XBOX
    
    RIGHT_X(4),
    RIGHT_Y(5),
    LEFT_X(0),
    LEFT_Y(1),
    

    //FANCY FLIGHT CONTROLLER
    /*
    RIGHT_X(3),
    RIGHT_Y(4),
    LEFT_X(0),
    LEFT_Y(1),
    */

    // RIGHT_X(3),
    // RIGHT_Y(4),
    // LEFT_X(0),
    // LEFT_Y(1),

    TUNER(6),
    LEFT_BACK(4),
    RIGHT_BACK(3);

    private final int id;

    Axis(int id) {
      this.id = id;
    }
  }

  public enum Shoulder {
    RIGHT_DOWN(2),
    LEFT_DOWN(4),
    LEFT_UP(5);

    private final int id;

    Shoulder(int id) {
      this.id = id;
    }
  }

  public enum Button {
    RESET(3),
    HAMBURGER(10),
    X(10),
    UP(10),
    DOWN(10);

    private final int id;

    Button(int id) {
      this.id = id;
    }
  }

  public enum Trim {
    LEFT_Y_POS(7),
    LEFT_Y_NEG(6),
    LEFT_X_POS(8),
    LEFT_X_NEG(9),
    RIGHT_X_POS(10),
    RIGHT_X_NEG(10),
    RIGHT_Y_POS(10),
    RIGHT_Y_NEG(10);

    private final int id;

    Trim(int id) {
      this.id = id;
    }
  }
}
