/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

import frc.robot.Constants;

public class LedStringSubsystem extends SubsystemBase {

  private AddressableLED mLed;
  private AddressableLEDBuffer mLedBuffer;
  // Store what the last hue of the first pixel is
  private int m_rainbowFirstPixelHue;
  private Constants.LedOption mLedOption;
  /**
   * Creates a new ExampleSubsystem.
   */
  public LedStringSubsystem() {
    // PWM port 9
    // Must be a PWM header, not MXP or DIO
    mLed = new AddressableLED(Constants.kPWM_LedSting);

    // Reuse buffer
    // Default to a length of 60, start empty output
    // Length is expensive to set, so only set it once, then just update data
    mLedBuffer = new AddressableLEDBuffer(60);
    mLed.setLength(mLedBuffer.getLength());

    // Set the data
    mLed.setData(mLedBuffer);
    mLed.start();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    option(mLedOption);
  }

  public void init() {
    // initialization stuff
    mLedOption = Constants.LedOption.RAINBOW;
  }

  private void rainbow() {
    // For every pixel
    for (var i = 0; i < mLedBuffer.getLength(); i++) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      final var hue = (m_rainbowFirstPixelHue + (i * 180 / mLedBuffer.getLength())) % 180;
      // Set the value
      mLedBuffer.setHSV(i, hue, 255, 128);
    }
    // Increase by to make the rainbow "move"
    m_rainbowFirstPixelHue += 3;
    // Check bounds
    m_rainbowFirstPixelHue %= 180;
  }

  public void option(Constants.LedOption select) {
    // Fill the buffer with selection
    switch (select) {
      case TEAM:
        break;
      case RED:
        break;
      case BLUE:
        break;
      case GREEN:
        break;
      case YELLOW:
        break;
      case ORANGE:
        break;
      case PURPLE:
        break;
      case RAINBOW:
        rainbow(); // Fill the buffer with a rainbow
        break;
    }
    // Set the LEDs
    mLed.setData(mLedBuffer);
  }
}
