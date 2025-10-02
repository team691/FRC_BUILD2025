// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Constants.TestConstants;
import frc.robot.subsystems.LimelightHelpers;
import frc.robot.subsystems.DriveTrain;
// import frc.robot.util.ScoreSafetyManager;

public class Test extends Command {
  private PIDController xController, yController, rotController;
//   private ProfiledPIDController rotControllerProfiled;
  // private boolean isRightScore;
  private Timer dontSeeTagTimer, stopTimer;
  private DriveTrain drivebase;
  private double tagID = -1;
  private double lastPoseValidatedTimestamp = -1;
//   private int dashboardLoopCounter = Math.max(0, Constants.DASHBOARD_UPDATE_PERIOD_CYCLES - 1);
  private boolean completionReported = false;

  private static final Test m_test = new Test(DriveTrain.getInstance());
  public static Test getInstance() {return m_test;}

//   private boolean shouldUpdateDashboard() {
//     if (!Constants.LIMIT_DASHBOARD_PERIODIC_UPDATES || Constants.DASHBOARD_UPDATE_PERIOD_CYCLES <= 1) {
//       return true;
//     }
//     dashboardLoopCounter++;
//     if (dashboardLoopCounter >= Constants.DASHBOARD_UPDATE_PERIOD_CYCLES) {
//       dashboardLoopCounter = 0;
//       return true;
//     }
//     return false;
//   }

  public Test(DriveTrain drivebase) {
    // Forward/back: raise this gain if the robot crawls toward the reef, lower if
    // it rockets past.
    xController = new PIDController(TestConstants.X_REEF_ALIGNMENT_P, 0.0, 0);
    // Strafe: tweak when the bot parks too far from the pole on the right side.
    yController = new PIDController(TestConstants.Y_REEF_ALIGNMENT_P, 0.0, 0);
    // Rotation: adjust when the bumper ends up angled inward/outward at the finish.
    rotController = new PIDController(TestConstants.ROT_REEF_ALIGNMENT_P, 0, 0);
    // Rotation
    // rotControllerProfiled = new
    // ProfiledPIDController(Constants.ROT_REEF_ALIGNMENT_P, 0, 0,
    // new TrapezoidProfile.Constraints(6.28, 3.14));
    // Rotation using holonic drive controller

    // var controller = new HolonomicDriveController(
    // new PIDController(Constants.X_REEF_ALIGNMENT_P, 0, 0), new
    // PIDController(Constants.Y_REEF_ALIGNMENT_P, 0, 0),
    // new ProfiledPIDController(Constants.ROT_REEF_ALIGNMENT_P, 0, 0,
    // new TrapezoidProfile.Constraints(6.28, 3.14)));
    // // Here, our rotation profile constraints were a max velocity
    // // of 1 rotation per second and a max acceleration of 180 degrees
    // // per second squared.
    // this.isRightScore = isRightScore;
    this.drivebase = drivebase;
    addRequirements(drivebase);
  }

  @Override
  public void initialize() {
    this.stopTimer = new Timer();
    this.stopTimer.start();
    this.dontSeeTagTimer = new Timer();
    this.dontSeeTagTimer.start();
    lastPoseValidatedTimestamp = -1;
    completionReported = false;

    SmartDashboard.putBoolean("AutoAlignRightComplete", false);

    rotController.setSetpoint(TestConstants.ROT_SETPOINT_REEF_ALIGNMENT);
    rotController.setTolerance(TestConstants.ROT_TOLERANCE_REEF_ALIGNMENT);

    // Adjust this X setpoint if we crash into or linger too far from the reef
    // (positive drives closer).
    xController.setSetpoint(TestConstants.X_SETPOINT_REEF_ALIGNMENT);
    xController.setTolerance(TestConstants.X_TOLERANCE_REEF_ALIGNMENT);

    // Nudge this Y setpoint when the right side scores off the mark (more positive
    // drifts toward the driver side).
    yController.setSetpoint(TestConstants.Y_R_SETPOINT_REEF_ALIGNMENT);
    yController.setTolerance(TestConstants.Y_TOLERANCE_REEF_ALIGNMENT);

    // Remember which tag we locked on so we stay tied to the correct reef face.
    tagID = LimelightHelpers.getFiducialID("limelight-left");
  }

  @Override
  public void execute() {
    // boolean updateDashboard = shouldUpdateDashboard();
    if (LimelightHelpers.getTV("limelight-left") && LimelightHelpers.getFiducialID("limelight-left") == tagID) {
      this.dontSeeTagTimer.reset();

      double[] postions = LimelightHelpers.getBotPose_TargetSpace("limelight-left");

      double xSpeed = -xController.calculate(postions[2]);
      double ySpeed = yController.calculate(postions[0]);
      double rotValue = rotController.calculate(postions[4]);

      boolean atPose = rotController.atSetpoint() && yController.atSetpoint() && xController.atSetpoint();
      if (atPose) {
        drivebase.drive(0.0, 0.0, 0.0, false, false);
        if (stopTimer.hasElapsed(TestConstants.POSE_VALIDATION_TIME)) {
          lastPoseValidatedTimestamp = Timer.getFPGATimestamp();
          if (!completionReported) {
            DriverStation.reportWarning("Auto align right finished", false);
            System.out.println("Auto align right finished");
            SmartDashboard.putBoolean("AutoAlignRightComplete", true);
            completionReported = true;
          }
        }
      } else {
        stopTimer.reset();
        lastPoseValidatedTimestamp = -1;
        // If the right-side approach veers left/right swap the sign of ySpeed or tune Y
        // setpoint/tolerance.
        // drivebase.drive(
        //     new Translation2d(
        //         // If we jump forward before we are centered, increase the Y tolerance gate or
        //         // lower this 0.03 safety creep.
        //         yController.getError() < 0.3 ? xSpeed : 0.00,
        //         ySpeed),
        //     rotValue,
        //     false);
        drivebase.drive(xSpeed, ySpeed, rotValue, false, false);
      }

    //   if (updateDashboard) {
    //     SmartDashboard.putNumber("xspeed", xSpeed);
    //   }
    } else {
      // drivebase.drive(
      // new Translation2d(),
      // 0,
      // false);
      drivebase.drive(0.0, 0.0, 0.0, false, false);
    //   if (updateDashboard) {
    //     SmartDashboard.putNumber("xspeed", 0);
    //   }
    }

    // if (updateDashboard) {
    //   SmartDashboard.putNumber("poseValidTimer", stopTimer.get());
    // }
  }

  @Override
  public void end(boolean interrupted) {
    drivebase.drive(0.0, 0.0, 0.0, false, false);
    // if (!interrupted) {
    //   ScoreSafetyManager.activateLockout(drivebase.getPose());
    // }
    if (!completionReported) {
      SmartDashboard.putBoolean("AutoAlignRightComplete", false);
    }
  }

  @Override
  public boolean isFinished() {
    // Requires the robot to stay in the correct position for 0.3 seconds, as long
    // as it gets a tag in the camera
    // Increase the wait time if intermittent vision causes premature cancel,
    // shorten to bail faster.
    double now = Timer.getFPGATimestamp();
    boolean poseRecentlyValidated = lastPoseValidatedTimestamp > 0
        && (now - lastPoseValidatedTimestamp) <= TestConstants.POSE_LOSS_GRACE_PERIOD;
    return poseRecentlyValidated
        || this.dontSeeTagTimer.hasElapsed(TestConstants.DONT_SEE_TAG_WAIT_TIME);
  }
}