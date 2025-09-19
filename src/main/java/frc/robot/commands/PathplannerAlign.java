// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.commands;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Constants;
import frc.robot.subsystems.LimelightHelpers;
import frc.robot.subsystems.LimelightHelpers.RawFiducial;
import frc.robot.subsystems.DriveTrain;

import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class PathplannerAlign extends Command {
 private PIDController xController, yController;
 private ProfiledPIDController thetaController;
 private boolean isRightScore;
 private Timer dontSeeTagTimer, stopTimer;
 private DriveTrain drivebase;
 private double tagID = -1;
 private HolonomicDriveController driveController;
 private PathPlannerTrajectoryState goalState;
 private Pose2d goalPose;

 public PathplannerAlign() {
    xController = new PIDController(Constants.X_REEF_ALIGNMENT_P, 0.0, 0.0);  // Vertical movement
    yController = new PIDController(Constants.Y_REEF_ALIGNMENT_P, 0.0, 0.0);  // Horitontal movement
    thetaController = new ProfiledPIDController(Constants.ROT_REEF_ALIGNMENT_P, 0.0, 0.0, new TrapezoidProfile.Constraints(Math.PI,  Math.PI)); // set null for testing asw
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    driveController = new HolonomicDriveController(xController, yController, thetaController);
 }

 @Override
 public void initialize() {
    tagID = LimelightHelpers.getFiducialID("limelight");

 }

 @Override
 public void execute() {
    double[] positions = LimelightHelpers.getBotPose_TargetSpace("limelight");

    Pose2d currentPose = new Pose2d(positions[0], positions[2], Rotation2d.fromDegrees(positions[4]));

    goalState = new PathPlannerTrajectoryState();
    goalPose = goalState.pose;

    // TODO: configure SwerveDriveOdometry + SwerveDrivePoseEstimator for real time robot pose update

    // DriveTrain.getInstance().drive(

    // )

    // TODO: use pathFindToPosition method in PathPlanner to generate path from current robot pose to apriltag pose, use PPSwerveController to execute path
 }

 @Override
 public void end(boolean interrupted) {
    DriveTrain.getInstance().drive(0, 0, 0, false, false);
 }

 @Override
 public boolean isFinished() {
  return true;
 }
}