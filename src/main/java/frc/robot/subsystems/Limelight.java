// package frc.robot.subsystems;


// import edu.wpi.first.networktables.NetworkTable;
// import edu.wpi.first.networktables.NetworkTableInstance;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;


// public class Limelight extends SubsystemBase {
//     private final NetworkTable tableInstance = NetworkTableInstance.getDefault().getTable("limelight");


//     // Fetch the horizontal offset ("tx") from the target
//     public double getYawError() {
//         return tableInstance.getEntry("tx").getDouble(0.0); // tx is the horizontal offset
//     }


//     // Check if the target is valid
//     public boolean hasValidTarget() {
//         return tableInstance.getEntry("tv").getDouble(0.0) == 1.0; // tv is 1.0 if a target is detected
//     }
// }

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
    private final NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    // Cached NetworkTable entries
    private final NetworkTableEntry tx = table.getEntry("tx"); // Horizontal offset
    private final NetworkTableEntry ty = table.getEntry("ty"); // Vertical offset
    private final NetworkTableEntry ta = table.getEntry("ta"); // Target area
    private final NetworkTableEntry tv = table.getEntry("tv"); // Valid target indicator

    // Fetch the horizontal offset ("tx") from the target
    public double getYawError() {
        return tx.getDouble(0.0); // Default to 0.0 if no data
    }

    // Fetch vertical offset ("ty") for distance estimation
    public double getVerticalOffset() {
        return ty.getDouble(0.0);
    }

    // Fetch target area ("ta"), which can be used for distance estimation
    public double getTargetArea() {
        return ta.getDouble(0.0);
    }

    // Check if the target is valid
    public boolean hasValidTarget() {
        return tv.getDouble(0.0) == 1.0;
    }

    // Optional: Estimate distance using ty (requires Limelight mounting angle & height)
    public double estimateDistance(double limelightHeight, double targetHeight, double limelightAngle) {
        if (!hasValidTarget()) return -1.0; // Return -1 if no valid target
        
        double angleToTarget = Math.toRadians(limelightAngle + getVerticalOffset());
        return (targetHeight - limelightHeight) / Math.tan(angleToTarget);
    }

    public double getDistance() {
        Rotation2d angleToGoal = Rotation2d.fromDegrees(LIMELIGHT_MOUNT_ANGLE)
            .plus(Rotation2d.fromDegrees(getYawError())); // Using yaw error for better alignment
        
        return (TARGET_HEIGHT - LIMELIGHT_HEIGHT) / angleToGoal.getTan(); // Distance estimation
    }    
}