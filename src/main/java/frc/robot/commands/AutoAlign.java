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
