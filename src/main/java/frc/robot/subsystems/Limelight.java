// package frc.robot.subsystems;

// import edu.wpi.first.networktables.NetworkTable;
// import edu.wpi.first.networktables.NetworkTableInstance;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// public class Limelight extends SubsystemBase {
//     private final NetworkTable tableInstance = NetworkTableInstance.getDefault().getTable("limelight");
//     private static final Limelight m_limelight = new Limelight();
//     public static Limelight getInstance() {
//         return m_limelight;
//     }
//     private Limelight() {}
//     // Fetch the horizontal offset ("tx") from the target
//     public double getYawError() {
//         return tableInstance.getEntry("tx").getDouble(0.0); // tx is the horizontal offset
//     }


//     // Fetch the forward/backward distance to the target using botpose
//     public double getForwardDistance() {
//         double[] botPose = LimelightHelpers.getBotPose_wpiRed("limelight");//tableInstance.getEntry("botpose").getDoubleArray(new double[6]);
        
//         System.out.println(botPose.length);

//         return (botPose.length >= 3) ? botPose[1] : 0.0; // botPose[1] is Y distance (forward/back)
//     }


//     // Fetch the left/right strafe distance (if needed)
//     public double getStrafeDistance() {
//         double[] botPose = tableInstance.getEntry("botpose").getDoubleArray(new double[6]);
//         return (botPose.length >= 3) ? botPose[0] : 0.0; // botPose[0] is X distance (left/right)
//     }


//     // Fetch yaw rotation relative to tag (for fine alignment)
//     public double getRobotYaw() {
//         double[] botPose = tableInstance.getEntry("botpose").getDoubleArray(new double[6]);
//         return (botPose.length >= 6) ? botPose[5] : 0.0; // botPose[5] is yaw rotation
//     }


//     // Check if the target is valid
//     public boolean hasValidTarget() {
//         return tableInstance.getEntry("tv").getDouble(0.0) == 1.0; // tv is 1.0 if a target is detected
//     }
// }

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.DriveTrain;

public class Limelight extends SubsystemBase {

  private DriveTrain m_drivetrain;
  private NetworkTable table;


  private double tx, ty, ta;
  private int tv;
  private double[] botpose;


  // tune these vals
  private static final double TX_KP = 0.02;  // Proportional control for rotation
  private static final double TY_KP = 0.05;  // Proportional control for forward/backward motion
  private static final double MIN_SPEED = 0.05; // Minimum speed to prevent dead zones

  public Limelight(DriveTrain driveTrain) {
    this.m_drivetrain = driveTrain;
    this.table = NetworkTableInstance.getDefault().getTable("limelight");
    this.tx = 0;
    this.ty = 0;
    this.ta = 0;
    this.tv = 0;
    this.botpose = new double[6];
  }


  @Override
  public void periodic() {
    updateLimelightVariables();
  }


  private void updateLimelightVariables() {
    tx = table.getEntry("tx").getDouble(0);
    ty = table.getEntry("ty").getDouble(0);
    ta = table.getEntry("ta").getDouble(0);
    tv = (int) table.getEntry("tv").getDouble(0);
    botpose = table.getEntry("botpose").getDoubleArray(new double[6]);
  }


  public void alignDrive() {
    if (tv == 0) {
      // No target detected, stop moving
      m_drivetrain.drive(0, 0, 0, true, false);
      System.out.println("it dont work");
      return;
    }


    double rotationSpeed = TX_KP * tx;
    if (Math.abs(rotationSpeed) < MIN_SPEED) {
      rotationSpeed = Math.copySign(MIN_SPEED, rotationSpeed); // Ensure minimum rotation
    }


    double forwardSpeed = -TY_KP * ty;
    if (Math.abs(forwardSpeed) < MIN_SPEED) {
      forwardSpeed = Math.copySign(MIN_SPEED, forwardSpeed);
    }


    double strafeSpeed = 0;


    m_drivetrain.drive(forwardSpeed, strafeSpeed, rotationSpeed, true, false);
  }


  public double getTx() { return tx; }
  public double getTy() { return ty; }
  public double getTa() { return ta; }
  public int getTv() { return tv; }
  public double[] getBotpose() { return botpose; }
}