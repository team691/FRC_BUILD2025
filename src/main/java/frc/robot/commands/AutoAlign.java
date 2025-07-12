// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.DriveTrain;
// import frc.robot.subsystems.Limelight;


// public class AutoAlign extends Command {
   
//     // Constants for PID tuning
//     private static final double kP_Yaw = 0.02; // Adjust for yaw correction
//     private static final double kP_Forward = 0.1; // Adjust for forward movement
//     private static final double kP_Strafe = 0.05; // Adjust for strafe correction
//     private static final double TARGET_DISTANCE = 0.5; // Stop at 0.5 meters from the tag

//     public AutoAlign() {
//         addRequirements(DriveTrain.getInstance());
//     }

//     @Override
//     public void execute() {
//         if (Limelight.getInstance().hasValidTarget()) {
//             // Get data from Limelight
//             double yawError = Limelight.getInstance().getYawError(); // Horizontal alignment
//             double forwardDistance = Limelight.getInstance().getForwardDistance(); // Distance to target
//             double strafeDistance = Limelight.getInstance().getStrafeDistance(); // Side alignment
//             // System.out.println("YawError: " + yawError + " Foward Dis: " +
//             //  forwardDistance + " StrafeDistance: " + strafeDistance);
            

//             // PID-based corrections
//             double rotationSpeed = -yawError * kP_Yaw;
//             double forwardSpeed = (forwardDistance > TARGET_DISTANCE) ? forwardDistance * kP_Forward : 0.0;
//             double strafeSpeed = -strafeDistance * kP_Strafe; // Strafe correction


//             // Drive the robot with swerve adjustments
//             //driveTrain.drive(forwardSpeed, strafeSpeed, rotationSpeed, true, false);
//         } else {
//             // No target detected, stop moving
//             System.out.println("Can't See");
//             //driveTrain.drive(0.0, 0.0, 0.0, true, false);
//         }
//     }

//     public void cancel(){
//         DriveTrain.getInstance().drive(0.0, 0.0, 0.0, true, false);
//     }
//     @Override
//     public void end(boolean interrupted) {
//         // Stop the drivetrain when toggled off
//         DriveTrain.getInstance().drive(0.0, 0.0, 0.0, true, false);
//     }


//     @Override
//     public boolean isFinished() {
//         // Stop when within target distance
//         return false;//limelight.getForwardDistance() < TARGET_DISTANCE;
//     }
// }

// TODO: move out robotpose from autoalign once functional, think about more pose estimations/paths to do (automatic game piece pick up, pose-based decision logic, autonavigation with pathplanner, etc.)

package frc.robot.commands;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.LimelightHelpers;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoAlign extends Command {
    private static final AutoAlign m_autoalign = new AutoAlign();
    public static AutoAlign getInstance() {return m_autoalign;}

    // private double[] targetpose = new double[6];

    double desiredX = 0.5; // meters (forward)
    double desiredY = 0.0; // meters (sideways)
    double desiredYaw = 0.0; // degrees
    
    public AutoAlign() {
        addRequirements(DriveTrain.getInstance());
    }

    public void execute() {
        LimelightHelpers.PoseEstimate pose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
        double[] currPose = LimelightHelpers.getBotPose_TargetSpace("limelight"); 
        // TODO: if necessary, calculate offset for limelight pose based on position
        if (pose.tagCount >= 1) {

            double errorX = currPose[0] - desiredX;
            double errorY = currPose[1] - desiredY;
            double errorYaw = currPose[5] - desiredYaw;

            // tune PID val
            double kP = 1.0;
            double driveCmd = -errorX * kP;
            double strafeCmd = -errorY * kP;
            double rotCmd = -errorYaw * kP;

            // minimize error
            if (Math.abs(errorX) < 0.05 && Math.abs(errorY) < 0.05 && Math.abs(errorYaw) < 3.0) {
                driveCmd = strafeCmd = rotCmd = 0;
            }

            DriveTrain.getInstance().drive(driveCmd, strafeCmd, rotCmd, false, true);            
        }
    }

    public boolean isFinished() {
        LimelightHelpers.PoseEstimate currPose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
        double[] currentPose = LimelightHelpers.getBotPose_TargetSpace("limelight");

        if (currPose.tagCount < 1) {
            return false;
        }

        double errorX = Math.abs(currentPose[0] - desiredX);
        double errorY = Math.abs(currentPose[1] - desiredY);
        double errorYaw = Math.abs(currentPose[5] - desiredYaw);

        return errorX < 0.05 && errorY < 0.05 && errorYaw < 3.0;
    }

    public void end(boolean interrupted) {
        DriveTrain.getInstance().drive(0, 0, 0, false, true);
    }
}