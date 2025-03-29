// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.ironmaple.simulation.SimulatedArena;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.ctre.phoenix.led.CANdle;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
// import frc.robot.subsystems.DriveTrain;
import frc.robot.enums.RobotMode;
import frc.robot.subsystems.BeamBreakers;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Shooter;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;
  private static final RobotMode JAVA_SIM_MODE = RobotMode.SIM;
  public static final RobotMode CURRENT_ROBOT_MODE = isReal() ? RobotMode.REAL : JAVA_SIM_MODE;

  private RobotContainer m_robotContainer;
  
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard. 

    // Set up data receivers & replay source
    
        // Set up data receivers & replay source
        switch (CURRENT_ROBOT_MODE) {
          case REAL:
              // Running on a real robot, log to a USB stick ("/U/logs")
              Logger.addDataReceiver(new WPILOGWriter());
              Logger.addDataReceiver(new NT4Publisher());
              break;

          case SIM:
              // Running a physics simulator, log to NT
              Logger.addDataReceiver(new NT4Publisher());
              break;

          case REPLAY:
              // Replaying a log, set up replay source
              //setUseTiming(false); // Run as fast as possible
              String logPath = LogFileUtil.findReplayLog();
              Logger.setReplaySource(new WPILOGReader(logPath));
              Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
              break;
      }

      // Start AdvantageKit logger
      Logger.start();
    m_robotContainer = new RobotContainer();
    
    Logger.start();
  }
  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    Logger.recordOutput("RobotPose", new Pose2d());
    
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    //m_lights.Red();
    //m_lights.redHue();
  }

  @Override
  public void disabledPeriodic() {
    //candle.setLEDs(255, 0, 0);
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector",
     * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
     * = new MyAutoCommand(); break; case "Default Auto": default:
     * autonomousCommand = new ExampleCommand(); break; }
     */

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    //candle.setLEDs(0, 0, 255);
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    //candle.setLEDs(0, 255, 0);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    //candle.setLEDs(0, 255, 0);
    if(BeamBreakers.getInstance().checkBeam()) {
      if(!Shooter.getInstance().isLaunched) {
        Shooter.getInstance().Score();
      }
    }
    //System.out.println(DriveTrain.getInstance().getHeading());
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
    //m_lights.Red();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    //m_lights.Red();
  }
  /** This function is called once when the robot is first started up. */
 /** This function is called once when the robot is first started up. */
 @Override
 public void simulationInit() {}

 /** This function is called periodically whilst in simulation. */
 @Override
 public void simulationPeriodic() {
     m_robotContainer.updateSimulation();
 }
}