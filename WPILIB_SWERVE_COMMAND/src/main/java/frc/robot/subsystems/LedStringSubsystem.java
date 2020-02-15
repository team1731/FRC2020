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
import edu.wpi.first.wpilibj.Timer;

import frc.robot.Constants.OpConstants;

public class LedStringSubsystem extends SubsystemBase {

  private Timer mTimer;
  private AddressableLED mLed;
  private AddressableLEDBuffer mLedBuffer;
  // Store what the last hue of the first pixel is
  private int m_rainbowFirstPixelHue;
  private int count;
  private double elapsed;
  private OpConstants.LedOption mLedOption;
  /**
   * Creates a new ExampleSubsystem.
   */
  public LedStringSubsystem() {
    // PWM port 9
    // Must be a PWM header, not MXP or DIO
    mLed = new AddressableLED(OpConstants.kPWM_LedSting);
    mTimer = new Timer();
    // Reuse buffer
    // Default to a length of 60, start empty output
    // Length is expensive to set, so only set it once, then just update data
    mLedBuffer = new AddressableLEDBuffer(60);
    mLed.setLength(mLedBuffer.getLength());

    // Set the data
    mLed.setData(mLedBuffer);
    mLed.start();
    count = 0;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (mTimer.get() - elapsed > 0.02) {
      option(mLedOption);
      elapsed = mTimer.get();
    }
  }

  public void init() {
    // initialization stuff
    mLedOption = OpConstants.LedOption.TEAM;
    mTimer.start();
    elapsed = mTimer.get();
  }

  private void fixedColor(int r, int g, int b) {
    // For every pixel
    int window = count + 8;
    for (var i = 0; i < mLedBuffer.getLength(); i++) {
      if ((i >= count) && (i < window)) { 
        mLedBuffer.setRGB(i, 0, 0, 0);
      } else {
        mLedBuffer.setRGB(i, r, g, b);
      }
    }
    count++;
    count %= mLedBuffer.getLength();
  }

  private void teamColors(int winSize) {
    int bufLen = mLedBuffer.getLength();
    int winMin;
    int winMax;

    if (count < winSize) {
      winMin = 0;
    } else {
      winMin = (count - winSize) % bufLen;
    }

    if (count < winSize) {
      winMax = count;
    } else {
      winMax = winMin + winSize;
      if (winMax > bufLen) winMax = bufLen;
    }

    // For every pixel
    for (var i = 0; i < mLedBuffer.getLength(); i++) {
      if ((i >= winMin) && (i < winMax)) { 
        mLedBuffer.setRGB(i, 255, 255, 0);
      } else {
        mLedBuffer.setRGB(i, 0, 0, 200);
      }
    }
    count++;
    count %= (bufLen + winSize);
  }

  private void rainbow() {
    // For every pixel
    for (var i = 0; i < mLedBuffer.getLength(); i++) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      final var hue = (m_rainbowFirstPixelHue + (i * 180 / mLedBuffer.getLength())) % 180;
      // Set the value
      mLedBuffer.setHSV(i, hue, 255, 128);
      //mLedBuffer.s
    }
    // Increase by to make the rainbow "move"
    m_rainbowFirstPixelHue += 3;
    // Check bounds
    m_rainbowFirstPixelHue %= 180;
  }

  public void option(OpConstants.LedOption select) {
    // Fill the buffer with selection
    switch (select) {
      case TEAM:
        teamColors(16);
        break;
      case RED:
        fixedColor(255, 0, 0);
        break;
      case BLUE:
        fixedColor(0, 0, 255);
        break;
      case GREEN:
        fixedColor(0, 200, 0);
        break;
      case YELLOW:
        fixedColor(255, 255, 0);
        break;
      case ORANGE:
        fixedColor(255, 128, 0);
        break;
      case PURPLE:
        fixedColor(75, 0, 150);
        break;
      case RAINBOW:
        rainbow(); // Fill the buffer with a rainbow
        break;
    }
    // Set the LEDs
    mLed.setData(mLedBuffer);
  }
}
