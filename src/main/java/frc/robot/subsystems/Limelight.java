package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
    private final NetworkTable tableInstance = NetworkTableInstance.getDefault().getTable("limelight");
    private static final Limelight m_limelight = new Limelight();
    public static Limelight getInstance() {
        return m_limelight;
    }
    private Limelight() {}
    // Fetch the horizontal offset ("tx") from the target
    public double getYawError() {
        return tableInstance.getEntry("tx").getDouble(0.0); // tx is the horizontal offset
    }


    // Fetch the forward/backward distance to the target using botpose
    public double getForwardDistance() {
        double[] botPose = LimelightHelpers.getBotPose_wpiRed("limelight");//tableInstance.getEntry("botpose").getDoubleArray(new double[6]);
        
        System.out.println(botPose.length);

        return (botPose.length >= 3) ? botPose[1] : 0.0; // botPose[1] is Y distance (forward/back)
    }


    // Fetch the left/right strafe distance (if needed)
    public double getStrafeDistance() {
        double[] botPose = tableInstance.getEntry("botpose").getDoubleArray(new double[6]);
        return (botPose.length >= 3) ? botPose[0] : 0.0; // botPose[0] is X distance (left/right)
    }


    // Fetch yaw rotation relative to tag (for fine alignment)
    public double getRobotYaw() {
        double[] botPose = tableInstance.getEntry("botpose").getDoubleArray(new double[6]);
        return (botPose.length >= 6) ? botPose[5] : 0.0; // botPose[5] is yaw rotation
    }


    // Check if the target is valid
    public boolean hasValidTarget() {
        return tableInstance.getEntry("tv").getDouble(0.0) == 1.0; // tv is 1.0 if a target is detected
    }
}
