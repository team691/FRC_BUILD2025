package frc.robot.commands;


// import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Limelight;

public class AutoAlign extends SubsystemBase {
    private final DriveTrain driveTrain;
    private final Limelight limelight;

    private final double yawKp = 0.03;
    private final double forwardKp = 0.1;
    private final double targetAreaThreshold = 2.5;
    private final double yawTolerance = 1.0;

    public AutoAlign(DriveTrain driveTrain, Limelight limelight) {
        this.driveTrain = driveTrain;
        this.limelight = limelight;
        addRequirements(driveTrain); // Only need DriveTrain for this command
    }

    // @Override
    // public void execute() {
    //     if (limelight.hasValidTarget()) {
    //         double yawError = limelight.getYawError(); // Get yaw offset
    //         double area = limelight.getTargetArea();

    //         double rotationSpeed = -yawError * 0.03; // Proportional control
    //         double forwardSpeed = (targetAreaThreshold - area) * forwardKp;

    //         rotationSpeed = Math.max(-0.5, Math.min(0.5, rotationSpeed));
    //         forwardSpeed = Math.max(0, Math.min(0.5, forwardSpeed));

    //         driveTrain.drive(forwardSpeed, 0.0, rotationSpeed, true, false); // Rotate only
    //         // return true;
    //     } else {
    //         driveTrain.drive(0.0, 0.0, 0.0, true, false); // Stop if no target
    //         // return false;
    //     }

    //     // return true;
    // }

    @Override
    public void execute() {
        if (limelight.hasValidTarget()) {
            double yawError = limelight.getYawError(); 
            double distance = limelight.getDistance();
            
            double rotationSpeed = -yawError * yawKp; 
            double forwardSpeed = (distance - TARGET_DISTANCE) * forwardKp;
            
            rotationSpeed = Math.max(-0.5, Math.min(0.5, rotationSpeed));
            forwardSpeed = Math.max(-0.5, Math.min(0.5, forwardSpeed));

            driveTrain.drive(forwardSpeed, 0.0, rotationSpeed, true, false);
        } else {
            driveTrain.drive(0.0, 0.0, 0.0, true, false);
        }
    }


    // @Override
    public void end(boolean interrupted) {
        // Stop the drivetrain when toggled off
        driveTrain.drive(0.0, 0.0, 0.0, true, false);
    }


    // @Override
    public boolean isFinished() {
        // Always return false since this command is toggled manually
        return false;
        return Math.abs(limelight.getYawError()) < yawTolerance && limelight.getTargetArea() >= targetAreaThreshold;
    }
}