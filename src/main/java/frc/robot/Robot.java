// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot
{

  private static Robot   instance;
  private        Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  private AddressableLED led = new AddressableLED(9);
  private AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(29);

  private Timer disabledTimer;

  private double ledTime = 0;
  private final double LED_DT = 1.0 / 60.0;

  private double[] m_blobPositions = {0.0, 23.0, 46.0, 69.0, 92.0};
  private double[] m_blobSpeeds    = {0.22, -0.17, 0.31, -0.26, 0.13};

  public Robot()
  {
    instance = this;
  }

  public static Robot getInstance()
  {
    return instance;
  }

  @Override
  public void robotInit()
  {
    m_robotContainer = new RobotContainer();

    disabledTimer = new Timer();

    led.setLength(ledBuffer.getLength());
    led.setData(ledBuffer);
    led.start();

    if (isSimulation())
    {
      DriverStation.silenceJoystickConnectionWarning(true);
    }
  }

  @Override
  public void robotPeriodic()
  {
    CommandScheduler.getInstance().run();
    
    updateLedsAuto();
  }

  private void updateLedsAuto() {
    ledTime += LED_DT;

    if (DriverStation.isDisabled()) {
      updateLedsDisabled();
    } else {
      boolean isRed = DriverStation.getAlliance()
          .map(a -> a == DriverStation.Alliance.Red)
          .orElse(false);
      updateLedsEnabled(isRed);
    }
  }

  private int clamp(int v) {
    return Math.max(0, Math.min(255, v));
  }

  @Override
  public void disabledInit()
  {
    m_robotContainer.setMotorBrake(true);
    disabledTimer.reset();
    disabledTimer.start();
  }

  @Override
  public void disabledPeriodic()
  {
    if (disabledTimer.hasElapsed(Constants.DrivebaseConstants.WHEEL_LOCK_TIME))
    {
      m_robotContainer.setMotorBrake(false);
      disabledTimer.stop();
      disabledTimer.reset();
    }
  }

  @Override
  public void autonomousInit()
  {
    m_robotContainer.setMotorBrake(true);
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    System.out.println("Auto selected: " + m_autonomousCommand);

    if (m_autonomousCommand != null)
    {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic()
  {
  }

  @Override
  public void teleopInit()
  {
    if (m_autonomousCommand != null)
    {
      m_autonomousCommand.cancel();
    } else
    {
      CommandScheduler.getInstance().cancelAll();
    }
  }

  @Override
  public void teleopPeriodic()
  {
  }

  @Override
  public void testInit()
  {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic()
  {
  }

  @Override
  public void simulationInit()
  {
  }

  @Override
  public void simulationPeriodic()
  {
  }
  private void updateLedsEnabled(boolean isRed) {
    int length = ledBuffer.getLength();
    for (int i = 0; i < length; i++) {
      // Calculate normalized distance from center (0.0 at center, 1.0 at ends)
      double centerDist = Math.abs(i - (length / 2.0)) / (length / 2.0);
      
      int r, g, b;
      if (isRed) {
        // Gradient from Light Red (center) to Deep Red (ends)
        r = (int) (255 - (centerDist * 155)); // 255 at center, 100 at ends
        g = 0;
        b = 0;
      } else {
        // Gradient from Light Blue (center) to Deep Blue (ends)
        r = 0;
        g = (int) (centerDist * 50); // Slight green tint at ends for "deep" look
        b = (int) (255 - (centerDist * 155)); // 255 at center, 100 at ends
      }
      ledBuffer.setRGB(i, clamp(r), clamp(g), clamp(b));
    }
    led.setData(ledBuffer);
  }

  private void updateLedsDisabled() {
    int length = ledBuffer.getLength();
    for (int i = 0; i < length; i++) {
      // Calculate normalized distance from center (0.0 at center, 1.0 at ends)
      double centerDist = Math.abs(i - (length / 2.0)) / (length / 2.0);

      // Gradient from Light Orange (center) to Deep Orange/Brown (ends)
      int r = (int) (255 - (centerDist * 155)); // 255 at center, 100 at ends
      int g = (int) (140 - (centerDist * 100)); // 140 at center, 40 at ends
      int b = 0;

      ledBuffer.setRGB(i, clamp(r), clamp(g), clamp(b));
    }
    led.setData(ledBuffer);
  }
}