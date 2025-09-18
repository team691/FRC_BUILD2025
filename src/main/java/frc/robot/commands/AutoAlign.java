package frc.robot.commands;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Constants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.LimelightHelpers;
import frc.robot.subsystems.LimelightHelpers.PoseEstimate;
import frc.robot.subsystems.Shooter;

public class AutoAlign extends Command {
    private final DriveTrain drivebase;
    private final boolean isRightScore;
    private final HolonomicDriveController driveController;
    private final ProfiledPIDController thetaController;

    private final Timer stopTimer = new Timer();
    private final Timer tagLostTimer = new Timer();

    public AutoAlign(boolean isRightScore, DriveTrain drivebase) {
        this.drivebase = drivebase;
        this.isRightScore = isRightScore;

        thetaController = new ProfiledPIDController(
            Constants.ROT_REEF_ALIGNMENT_P, 0.0, 0.0,
            new TrapezoidProfile.Constraints(Math.PI, Math.PI)
        );
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        driveController = new HolonomicDriveController(
            new edu.wpi.first.math.controller.PIDController(Constants.X_REEF_ALIGNMENT_P, 0.0, 0.0),
            new edu.wpi.first.math.controller.PIDController(Constants.Y_REEF_ALIGNMENT_P, 0.0, 0.0),
            thetaController
        );

        addRequirements(drivebase);
    }

    @Override
    public void initialize() {
        stopTimer.reset();
        stopTimer.start();
        tagLostTimer.reset();
        tagLostTimer.start();
    }

    @Override
    public void execute() {
        if (!LimelightHelpers.getTV("limelight")) {
            // No tag seen
            drivebase.drive(0, 0, 0, false, false);
            return;
        }

        tagLostTimer.reset();  // reset lost tag timer because we see a tag

        // Use robot pose relative to target — more stable
        PoseEstimate currentPose = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");

        // Target pose is fixed relative to tag
        Pose2d targetPose = new Pose2d(
            Constants.X_SETPOINT_REEF_ALIGNMENT,
            isRightScore ? Constants.Y_SETPOINT_REEF_ALIGNMENT : -Constants.Y_SETPOINT_REEF_ALIGNMENT,
            Rotation2d.fromDegrees(Constants.ROT_SETPOINT_REEF_ALIGNMENT)
        );

        Trajectory.State dummyState = new Trajectory.State(0, 0, 0, targetPose, 0);

        ChassisSpeeds speeds = driveController.calculate(currentPose.pose, dummyState, targetPose.getRotation());

        drivebase.drive(
            speeds.vxMetersPerSecond,
            speeds.vyMetersPerSecond,
            speeds.omegaRadiansPerSecond,
            false,
            false
        );

        SmartDashboard.putNumber("AlignTimer", stopTimer.get());

        // If aligned to Tag ID 7 and all controllers are within tolerance, shoot
        double visibleID = LimelightHelpers.getFiducialID("limelight");

        boolean aligned = driveController.atReference();
        if (aligned && visibleID == 7) {
            Shooter.getInstance().shootTest(Constants.ShooterConstants.ShooterPower);
        } else {
            Shooter.getInstance().stopShoot();
        }

        // Reset stop timer if not aligned
        if (!aligned) {
            stopTimer.reset();
        }
    }

    @Override
    public void end(boolean interrupted) {
        drivebase.drive(0, 0, 0, false, false);
        Shooter.getInstance().stopShoot();
    }

    @Override
    public boolean isFinished() {
        return tagLostTimer.hasElapsed(Constants.DONT_SEE_TAG_WAIT_TIME)
            || stopTimer.hasElapsed(Constants.POSE_VALIDATION_TIME);
    }
}
