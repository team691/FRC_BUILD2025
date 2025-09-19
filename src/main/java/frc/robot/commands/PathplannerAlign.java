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
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class PathplannerAlign extends Command {
 private PIDController xController, yController, rotController;
 private ProfiledPIDController thetaController;
 private boolean isRightScore;
 private Timer dontSeeTagTimer, stopTimer;
 private DriveTrain drivebase;
 private double tagID = -1;
 private HolonomicDriveController driveController;


 public PathplannerAlign(boolean isRightScore, DriveTrain drivebase) {
   xController = new PIDController(Constants.X_REEF_ALIGNMENT_P, 0.0, 0);  // Vertical movement
   yController = new PIDController(Constants.Y_REEF_ALIGNMENT_P, 0.0, 0);  // Horitontal movement
   rotController = new PIDController(Constants.ROT_REEF_ALIGNMENT_P, 0, 0);  // Rotation


   thetaController = new ProfiledPIDController(Constants.ROT_REEF_ALIGNMENT_P, 0, 0, new TrapezoidProfile.Constraints(Math.PI,  Math.PI)); // set null for testing asw
   thetaController.enableContinuousInput(-Math.PI, Math.PI);
   driveController = new HolonomicDriveController(xController, yController, thetaController);


   this.isRightScore = isRightScore;
   this.drivebase = drivebase;
   addRequirements(drivebase);
 }
  @Override
 public void initialize() {
   this.stopTimer = new Timer();
   this.stopTimer.start();
   this.dontSeeTagTimer = new Timer();
   this.dontSeeTagTimer.start();


   rotController.setSetpoint(Constants.ROT_SETPOINT_REEF_ALIGNMENT);
   rotController.setTolerance(Constants.ROT_TOLERANCE_REEF_ALIGNMENT);


   xController.setSetpoint(Constants.X_SETPOINT_REEF_ALIGNMENT);
   xController.setTolerance(Constants.X_TOLERANCE_REEF_ALIGNMENT);


   yController.setSetpoint(isRightScore ? Constants.Y_SETPOINT_REEF_ALIGNMENT : -Constants.Y_SETPOINT_REEF_ALIGNMENT);
   yController.setTolerance(Constants.Y_TOLERANCE_REEF_ALIGNMENT);


   tagID = LimelightHelpers.getFiducialID("limelight");
   double pls = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
   System.out.println("pls" + pls);


   RawFiducial[] fiducials = LimelightHelpers.getRawFiducials("limelight");
   for (RawFiducial fiducial : fiducials) {
       int id = fiducial.id;                    // Tag ID
       double txnc = fiducial.txnc;             // X offset (no crosshair)
       double tync = fiducial.tync;             // Y offset (no crosshair)
       double ta = fiducial.ta;                 // Target area
       double distToCamera = fiducial.distToCamera;  // Distance to camera
       double distToRobot = fiducial.distToRobot;    // Distance to robot
       double ambiguity = fiducial.ambiguity;   // Tag pose ambiguity
       System.out.print("test");
       System.out.print("id" + id);
   }
 }


 @Override
 public void execute() {
   if (LimelightHelpers.getTV("limelight") && LimelightHelpers.getFiducialID("limelight") == tagID) {
     this.dontSeeTagTimer.reset();


     double[] postions = LimelightHelpers.getBotPose_TargetSpace("limelight");
     SmartDashboard.putNumber("x", postions[2]);
     System.out.println("x" + postions[2]);


     Pose2d currentPose = new Pose2d(
       postions[0],
       postions[2],
       Rotation2d.fromDegrees(postions[4])
     );


     Pose2d targetPose = new Pose2d(
       isRightScore ? Constants.Y_SETPOINT_REEF_ALIGNMENT : -Constants.Y_SETPOINT_REEF_ALIGNMENT,
       Constants.X_SETPOINT_REEF_ALIGNMENT,
       Rotation2d.fromDegrees(Constants.ROT_SETPOINT_REEF_ALIGNMENT)
     );


     Trajectory.State targetState = new Trajectory.State(
       0.0,
       0.0,
       0.0,
       targetPose,
       0.0
     );


     ChassisSpeeds speeds = driveController.calculate(currentPose, targetState, targetPose.getRotation());


     double xSpeed = xController.calculate(postions[2]);
     SmartDashboard.putNumber("xspeed", xSpeed);
     double ySpeed = -yController.calculate(postions[0]);
     double rotValue = -rotController.calculate(postions[4]);


    //  drivebase.drive(xSpeed, ySpeed, rotValue, false, false);


     drivebase.drive(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond, false, false);


     if (!rotController.atSetpoint() ||
         !yController.atSetpoint() ||
         !xController.atSetpoint()) {
       stopTimer.reset();
     }
   } else {
     drivebase.drive(0.0, 0.0, 0, false, false);
   }


   SmartDashboard.putNumber("poseValidTimer", stopTimer.get());
 }


 @Override
 public void end(boolean interrupted) {
   drivebase.drive(0.0, 0.0, 0, false, false);
 }


 @Override
 public boolean isFinished() {
   // Requires the robot to stay in the correct position for 0.3 seconds, as long as it gets a tag in the camera
   return this.dontSeeTagTimer.hasElapsed(Constants.DONT_SEE_TAG_WAIT_TIME) ||
       stopTimer.hasElapsed(Constants.POSE_VALIDATION_TIME);
 }
}