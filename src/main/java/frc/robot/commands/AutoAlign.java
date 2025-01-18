package frc.robot.commands;


// import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Limelight;


public class AutoAlign extends Command {
    private final DriveTrain driveTrain;
    private final Limelight limelight;


    public AutoAlign(DriveTrain driveTrain, Limelight limelight) {
        this.driveTrain = driveTrain;
        this.limelight = limelight;
        addRequirements(driveTrain); // Only need DriveTrain for this command
    }


    @Override
    public void execute() {
        if (limelight.hasValidTarget()) {
            double yawError = limelight.getYawError(); // Get yaw offset
            double rotationSpeed = -yawError * 0.03; // Proportional control
            driveTrain.drive(0.0, 0.0, rotationSpeed, true, false); // Rotate only
        } else {
            driveTrain.drive(0.0, 0.0, 0.0, true, false); // Stop if no target
        }
    }


    @Override
    public void end(boolean interrupted) {
        // Stop the drivetrain when toggled off
        driveTrain.drive(0.0, 0.0, 0.0, true, false);
    }


    @Override
    public boolean isFinished() {
        // Always return false since this command is toggled manually
        return false;
    }
}